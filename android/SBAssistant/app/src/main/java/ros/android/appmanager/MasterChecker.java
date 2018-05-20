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

import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.namespace.GraphName;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.Date;
import java.util.List;

import ros.android.util.RobotDescription;

/**
 * Threaded ROS-master checker. Runs a thread which checks for a valid ROS
 * master on the remote and sends back a {@link RobotDescription} (with robot name and type)
 * on success or a failure reason on failure.
 *
 * Note that after an application restart on the robot, it can take up to
 * several seconds for the application to initialize.
 *
 * Do not allow more than one thread to execute at a time. Ignore subsequent requests.
 * On a error the thread may show an error dialog.
 * 
 * @author hersh@willowgarage.com
 */
public class MasterChecker {
    private final static String CLSS = "MasterChecker";
	private CheckerThread checkerThread;
	private final SBRobotConnectionHandler handler;
    private RobotDescription robot = null;
    private int attemptLimit = 1;
    private boolean threadRunning;

	/** Constructor. Should not take any time. */
	public MasterChecker(SBRobotConnectionHandler h) {
        this.threadRunning = false;
	    this.handler = h;
	}

	/**
	 * Start the checker thread with the main robot description. If the thread is already
	 * running, kill it first and then start anew. Returns immediately.
	 */
	public void beginChecking(RobotDescription rbt,int limit) {
	    if( this.threadRunning ) {
            Log.i(CLSS, "check already in progress ...");
            return;
        }
	    this.attemptLimit = limit;
        Log.i(CLSS, "Attempting to connect ...");

		// Validate the master URI
		String masterUri = rbt.getRobotId().getMasterUri();
		if(masterUri == null) {
            Log.i(CLSS, "Empty URI ...");
			handler.handleRobotCommunicationError("empty master URI");
			return;
		}
		URI uri;
		try {
			uri = new URI(masterUri);
		}
		catch(URISyntaxException e) {
			handler.handleRobotCommunicationError("invalid master URI");
			return;
		}
		checkerThread = new CheckerThread(rbt,uri);
		checkerThread.start();
	}

	/** Stop the checker thread.
     *
     */
	public void stopChecking() {
		if(checkerThread != null && checkerThread.isAlive()) {
			checkerThread.interrupt();
		}
	}

	public RobotDescription getRobot() { return this.robot; }


	private class CheckerThread extends Thread {

        private URI masterUri;
        private RobotDescription robot;

        public CheckerThread(RobotDescription rbt,URI uri) {
            this.robot = rbt;
            this.masterUri = uri;

            setDaemon(true);

            // don't require callers to explicitly kill all the old checker threads.
            setUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() {
                @Override
                public void uncaughtException(Thread thread, Throwable ex) {
                    Log.e(CLSS, String.format("Uncaught exception connecting: %s",ex.getLocalizedMessage()),ex);
                    handler.handleRobotCommunicationError("Exception: " + ex.getMessage());
                    threadRunning = false;
                }
            });
        }

        @Override
        public void run() {
            threadRunning = true;
            int attempt = 0;
            while (attempt < attemptLimit) {
                try {
                    Log.i(CLSS, "Getting parameters ......");
                    ParameterClient paramClient = new ParameterClient(new NodeIdentifier(GraphName.of("/master_checker"), masterUri), masterUri);
                    boolean hasName = ((Boolean) paramClient.hasParam(GraphName.of("robot/name")).getResult()).booleanValue();
                    boolean hasType = ((Boolean) paramClient.hasParam(GraphName.of("robot/type")).getResult()).booleanValue();
                    boolean hasApp = ((Boolean) paramClient.hasParam(GraphName.of("robot/application")).getResult()).booleanValue();
                    // Log the names
                    Log.i(CLSS, "Parameters ......");
                    List<GraphName> names = paramClient.getParamNames().getResult();
                    for (GraphName name : names) {
                        try {
                            Log.i(CLSS, String.format("   %s = %s", name.toString(), paramClient.getParam(name).getResult().toString()));
                        }
                        catch(Exception ex) {
                            Log.i(CLSS, String.format("EXCEPTION: getting parameter %s (%s)", name.toString(), ex.getLocalizedMessage()));
                        }
                    }

                    if (hasName && hasType) {
                        robot.setRobotName(paramClient.getParam(GraphName.of("robot/name")).getResult().toString());
                        robot.setRobotType(paramClient.getParam(GraphName.of("robot/type")).getResult().toString());
                        if (((Boolean) paramClient.hasParam(GraphName.of("robot/platform")).getResult()).booleanValue()) {
                            robot.setPlatform((String) paramClient.getParam(GraphName.of("robot/platform")).getResult());
                        }
                        robot.setTimeLastSeen(new Date());
                        handler.receiveRobotConnection();

                        if (hasApp) {
                            String appName = (String) paramClient.getParam(GraphName.of("robot/application")).getResult();
                            robot.setApplicationName(appName);
                            handler.receiveApplication(appName);
                        }
                    }
                    else {
                        Log.e(CLSS, "No parameters");
                        handler.handleRobotCommunicationError("The parameters on the server are not set. Please set robot/name and robot/type.");
                    }
                    threadRunning = false;
                    return;
                }
                catch (XmlRpcTimeoutException tex) {
                    if (attempt < attemptLimit) {
                        attempt++;
                        continue;
                    }
                    Log.e(CLSS, "Timeout Exception while creating parameter client in MasterChecker for " + masterUri);
                    handler.handleRobotCommunicationError(tex.getLocalizedMessage());
                }
                catch (Throwable ex) {
                    Log.e(CLSS, "Exception while creating parameter client in MasterChecker for master URI " + masterUri, ex);
                    handler.handleRobotCommunicationError(ex.getLocalizedMessage());
                    break;
                }
            }
            threadRunning = false;
        }
    }
}
