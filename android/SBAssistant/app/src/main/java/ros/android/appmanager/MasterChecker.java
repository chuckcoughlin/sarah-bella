/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package ros.android.appmanager;

import android.util.Log;
import android.view.View;
import android.widget.EditText;

import org.ros.address.InetAddressFactory;
import org.ros.android.NodeMainExecutorService;
import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.namespace.GraphName;
import org.ros.node.NodeConfiguration;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import chuckcoughlin.sb.assistant.R;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

/**
 * Threaded ROS-master checker. Runs a thread which checks for a valid ROS
 * master and sends back a {@link RobotDescription} (with robot name and type)
 * on success or a failure reason on failure.
 * 
 * @author hersh@willowgarage.com
 */
public class MasterChecker {
    private final static String CLSS = "MasterChecker";
	private CheckerThread checkerThread;
	private final SBRobotConnectionHandler handler;

	/** Constructor. Should not take any time. */
	public MasterChecker(SBRobotConnectionHandler h) {
		this.handler = h;
	}

	/**
	 * Start the checker thread with the given robotId. If the thread is already
	 * running, kill it first and then start anew. Returns immediately.
	 */
	public void beginChecking(RobotDescription robot) {
		stopChecking();
		if(robot.getRobotId().getMasterUri() == null) {
			handler.handleConnectionError("empty master URI");
			return;
		}
		URI uri;
		try {
			uri = new URI(robot.getRobotId().getMasterUri());
		}
		catch(URISyntaxException e) {
			handler.handleConnectionError("invalid master URI");
			return;
		}

		checkerThread = new CheckerThread(robot, uri);
		checkerThread.start();
	}

	/** Stop the checker thread. */
	public void stopChecking() {
		if(checkerThread != null && checkerThread.isAlive()) {
			checkerThread.interrupt();
		}
	}

	private class CheckerThread extends Thread {

        private URI masterUri;
        private RobotDescription robot;

        public CheckerThread(RobotDescription robotDescription, URI masterUri) {
            this.masterUri = masterUri;
            this.robot = robotDescription;

            setDaemon(true);

            // don't require callers to explicitly kill all the old checker
            // threads.
            setUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() {
                @Override
                public void uncaughtException(Thread thread, Throwable ex) {
                    handler.handleConnectionError("exception: " + ex.getMessage());
                }
            });
        }

        @Override
        public void run() {
            try {
                ParameterClient paramClient = new ParameterClient(new NodeIdentifier(GraphName.of("/master_checker"), masterUri), masterUri);
                boolean hasName = ((Boolean) paramClient.hasParam(GraphName.of("robot/name")).getResult()).booleanValue();
                boolean hasType = ((Boolean) paramClient.hasParam(GraphName.of("robot/type")).getResult()).booleanValue();
                boolean hasApp = ((Boolean) paramClient.hasParam(GraphName.of("robot/application")).getResult()).booleanValue();
                // Log the names
                Log.i(CLSS, "Parameters ......");
                List<GraphName> names = paramClient.getParamNames().getResult();
                for (GraphName name : names) {
                    Log.i(CLSS, String.format("   %s", name.toString()));
                }

                if (hasName && hasType) {
                    robot.setRobotName(paramClient.getParam(GraphName.of("robot/name")).getResult().toString());
                    robot.setRobotType(paramClient.getParam(GraphName.of("robot/type")).getResult().toString());
                    robot.setPlatform((String) paramClient.getParam(GraphName.of("robot/type")).getResult());
                    robot.setTimeLastSeen(new Date());
                    handler.receiveConnection(robot);

                    if (hasApp) {
                        handler.receiveApplication((String) paramClient.getParam(GraphName.of("robot/application")).getResult());
                    }
                } else {
                    Log.e(CLSS, "No parameters");
                    handler.handleConnectionError("The parameters on the server are not set. Please set robot/name and robot/type.");
                }
                return;
            } catch (Throwable ex) {
                Log.e(CLSS, "Exception while creating node in MasterChecker for master URI " + masterUri, ex);
                handler.handleConnectionError(ex.getLocalizedMessage());
            }
        }
    }

}
