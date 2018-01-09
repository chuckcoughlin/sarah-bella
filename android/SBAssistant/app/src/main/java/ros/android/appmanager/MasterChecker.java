/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2011, Willow Garage, Inc.
* Copyright (c) 2013, OSRF.
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
 * See: http://docs.ros.org/hydro/api/android_apps/html/MasterChecker_8java_source.html
 */

package ros.android.appmanager;

import android.util.Log;

import org.ros.address.InetAddressFactory;
import org.ros.android.NodeMainExecutorService;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.namespace.GraphName;
import org.ros.node.NodeConfiguration;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.Date;

import app_manager.Icon;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;


public class MasterChecker {
    private final static String CLSS = "MasterChecker";

    private CheckerThread checkerThread;
    private SBRobotConnectionHandler handler;

    public MasterChecker(SBRobotConnectionHandler handler) {
        this.handler = handler;
    }

    public void beginChecking(RobotId robotId) {
        stopChecking();
      if (robotId.getMasterUri() == null) {
             handler.handleConnectionError("empty master URI");
             return;
         }
         URI uri;
         try {
             uri = new URI(robotId.getMasterUri());
         } catch (URISyntaxException e) {
             handler.handleConnectionError("invalid master URI");
             return;
         }
         checkerThread = new CheckerThread(robotId, uri);
         checkerThread.start();
     }

     public void stopChecking() {
         if (checkerThread != null && checkerThread.isAlive()) {
             checkerThread.interrupt();
         }
     }

     private class CheckerThread extends Thread {
         private URI masterUri;
         private RobotId robotId;

         public CheckerThread(RobotId robotId, URI masterUri) {
             this.masterUri = masterUri;
             this.robotId = robotId;
             setDaemon(true);
             // don't require callers to explicitly kill all the old checker threads.
             setUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() {
                 @Override
                 public void uncaughtException(Thread thread, Throwable ex) {
                     String reason = String.format("MasterChecker: Exception %s", ex.getLocalizedMessage());
                     handler.handleConnectionError(reason);
                 }
             });
         }

         @Override
         public void run() {
             try {
                 // Check if the master exists - no really good way in rosjava except by checking a standard parameter.
                 ParameterClient paramClient = new ParameterClient(
                         NodeIdentifier.forNameAndUri("/master_checker", masterUri.toString()), masterUri);
                 // getParam throws when it can't find the parameter.
                 String unused_rosversion = (String) paramClient.getParam(GraphName.of("rosversion")).getResult();

                 // Check for the platform information - be sure to check that master exists first otherwise you'll
                 // start a thread which perpetually crashes and triest to re-register in .execute()
                 NodeMainExecutorService nodeMainExecutorService = new NodeMainExecutorService();
                 NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(
                         InetAddressFactory.newNonLoopback().getHostAddress(),
                         masterUri);
                 GatewayInfoSubscriber gatewayInfoClient = new GatewayInfoSubscriber();
                 nodeMainExecutorService.execute(gatewayInfoClient, nodeConfiguration.setNodeName("gateway_info_client_node"));
                 gatewayInfoClient.waitForResponse();
                 String gatewayName = gatewayInfoClient.getGatewayName();
                 PlatformInfoServiceClient client = new PlatformInfoServiceClient();
                 nodeMainExecutorService.execute(client, nodeConfiguration.setNodeName("platform_info_client_node"));
                 client.waitForResponse();
                 String robotName = client.getRobotUniqueName();
                 String robotType = client.getRobotType();
                 Icon robotIcon = client.getRobotIcon();
                 StatusServiceClient statusClient = new StatusServiceClient(client.getRobotAppManagerNamespace(), gatewayName);
                 nodeMainExecutorService.execute(statusClient, nodeConfiguration.setNodeName("status_client_node"));
                 statusClient.waitForResponse();
                 nodeMainExecutorService.shutdownNodeMain(client);
                 nodeMainExecutorService.shutdownNodeMain(gatewayInfoClient);
                 nodeMainExecutorService.shutdownNodeMain(statusClient);

                 // configure robot description
                 Date timeLastSeen = new Date();
                 RobotDescription robotDescription = new RobotDescription(robotId, robotName, robotType, robotIcon, gatewayName,
                         timeLastSeen);
                 if (statusClient.isAvailable()) {
                     Log.i(CLSS, "rapp manager is available");
                     robotDescription.setConnectionStatus(RobotDescription.OK);
                 } else {
                     Log.i(CLSS, "rapp manager is unavailable");
                     robotDescription.setConnectionStatus(RobotDescription.CONNECTION_STATUS_UNAVAILABLE);
                 }
                 handler.receiveConnection(robotDescription);
                 return;
             }
             catch ( java.lang.RuntimeException e) {
                 // thrown if master could not be found in the getParam call (from java.net.ConnectException)
                 Log.w(CLSS, "could not find the master [" + masterUri + "][" + e.toString() + "]");
                 handler.handleConnectionError(e.getLocalizedMessage());
             }
             catch (ServiceNotFoundException e) {
                 // thrown by client.waitForResponse() if it times out
                 Log.w(CLSS, e.getMessage()); // e.getMessage() is a little less verbose (no org.ros.exception.ServiceNotFoundException prefix)
                 handler.handleConnectionError(e.getMessage());  // don't need the master uri, it's already shown above in the robot description from input method.
             }
             catch (Throwable e) {
                 Log.w(CLSS, "exception while creating node in masterchecker for master URI "
                         + masterUri, e);
                 handler.handleConnectionError(e.getLocalizedMessage());
             }
         }
  }
 }

