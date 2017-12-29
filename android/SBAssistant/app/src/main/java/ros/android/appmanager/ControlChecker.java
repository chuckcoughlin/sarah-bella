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
 *
 * See: http://docs.ros.org/hydro/api/android_apps/html/ControlChecker_8java_source.html
 */
package ros.android.appmanager;

import android.util.Log;

import org.apache.http.HttpResponse;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.DefaultHttpClient;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.URI;

import ros.android.util.RobotId;

public class ControlChecker {
    public interface SuccessHandler {
        void handleSuccess();
    }

    public interface FailureHandler {
        void handleFailure(String reason);
    }

    public interface EvictionHandler {
        boolean doEviction(String user);
    }

    public interface StartHandler {
        void handleStarting();
    }

    private CheckerThread checkerThread;
    private SuccessHandler robotReadyCallback;
    private FailureHandler failureCallback;
    private EvictionHandler evictionCallback;
    private StartHandler startCallback;
    private boolean doStart;

    public ControlChecker(SuccessHandler robotReadyCallback, FailureHandler failureCallback) {
        this.robotReadyCallback = robotReadyCallback;
        this.failureCallback = failureCallback;
        this.evictionCallback = new EvictionHandler() {
            public boolean doEviction(String user) {
                return false;
            }
        };
        this.startCallback = null;
        this.doStart = false;
    }

    public ControlChecker(SuccessHandler robotReadyCallback, FailureHandler failureCallback, EvictionHandler evictionCallback) {
        this.robotReadyCallback = robotReadyCallback;
        this.failureCallback = failureCallback;
        this.evictionCallback = evictionCallback;
        this.startCallback = null;
        this.doStart = true;
    }

    public ControlChecker(SuccessHandler robotReadyCallback, FailureHandler failureCallback, EvictionHandler evictionCallback, StartHandler startCallback) {
        this.robotReadyCallback = robotReadyCallback;
        this.failureCallback = failureCallback;
        this.evictionCallback = evictionCallback;
        this.startCallback = startCallback;
        this.doStart = true;
    }

    public void beginChecking(RobotId robotId) {
        stopChecking();
        //If there's no wifi tag in the robot id, skip this step
        if (robotId.getControlUri() == null) {
            robotReadyCallback.handleSuccess();
            return;
        }
        checkerThread = new CheckerThread(robotId);
        checkerThread.start();
    }

    public void stopChecking() {
        if (checkerThread != null && checkerThread.isAlive()) {
            checkerThread.interrupt();
        }
    }

    private class CheckerThread extends Thread {
        private RobotId robotId;

        public CheckerThread(RobotId robotId) {
            this.robotId = robotId;
            setDaemon(true);
            // don't require callers to explicitly kill all the old checker threads.
            setUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() {
                @Override
                public void uncaughtException(Thread thread, Throwable ex) {
                    failureCallback.handleFailure("exception: " + ex.getMessage());
                }
            });
        }

        private String getPage(String uri) {
            try {
                HttpClient client = new DefaultHttpClient();
                HttpGet request = new HttpGet();
                request.setURI(new URI(uri));
                HttpResponse response = client.execute(request);
                BufferedReader in = new BufferedReader
                        (new InputStreamReader(response.getEntity().getContent()));
                StringBuffer sb = new StringBuffer("");
                String line = "";
                String NL = System.getProperty("line.separator");
                while ((line = in.readLine()) != null) {
                    sb.append(line + NL);
                }
                in.close();
                String page = sb.toString();
                return page;
            } catch (java.io.IOException ex) {
                Log.e("ControlChecker", "IOError: " + uri, ex);
            } catch (java.net.URISyntaxException ex) {
                Log.e("ControlChecker", "URI Invalid: " + uri, ex);
            }
            return null;
        }

        final String USER_TAG = "ACTIVE_USER:";
        final String VALID_USER = "applications";
        final String NO_USER = "None";

        private String getActiveUser() {
            String page = getPage(robotId.getControlUri() + "?action=GET_STATE");
            if (page == null) {
                return null;
            }
            String[] pageLines = page.split("\n");
            String activeUser = NO_USER;

            for (String i : pageLines) {
                if (i.trim().indexOf(USER_TAG) >= 0) {
                    activeUser = i.trim().substring(USER_TAG.length() + 1).trim();
                }
            }
            return activeUser;
        }

        @Override
        public void run() {
            try {
                String activeUser = getActiveUser();
                Log.d("ControlChecker", "Active user: " + activeUser);
                if (activeUser == null) {
                    failureCallback.handleFailure("Could not connect to the control page");
                    return;
                }
                boolean goodState = false;
                boolean badUser = false;
                if (activeUser.equals(VALID_USER)) {
                    goodState = true;
                } else if (!activeUser.equals(NO_USER)) {
                    badUser = true;
                }
                if (goodState) {
                    robotReadyCallback.handleSuccess();
                } else {
                    if (badUser) {
                        if (evictionCallback.doEviction(activeUser)) { //Prompt
                            Log.d("ControlChecker", "Stopping robot");
                            getPage(robotId.getControlUri() + "?action=STOP_ROBOT");
                        } else {
                            failureCallback.handleFailure("Need to evict current user inorder to connect");
                            return;
                        }
                    }
                    if (doStart) {
                        if (startCallback != null) {
                            startCallback.handleStarting();
                        }
                        Log.d("ControlChecker", "Starting robot");
                        getPage(robotId.getControlUri() + "?action=START_ROBOT");

                        int i = 0;

                        while (i < 30 && !VALID_USER.equals(getActiveUser())) {
                            i++;

                        }

                        if (VALID_USER.equals(getActiveUser())) {
                            robotReadyCallback.handleSuccess();
                        } else {
                            failureCallback.handleFailure("Re-started the robot, but it is still not working");
                        }
                    } else {
                        //Non-started robot
                        failureCallback.handleFailure("Robot not started");
                    }
                }
            } catch (Throwable ex) {
                Log.e("ControlChecker", "Exception while checking control URI "
                        + robotId.getControlUri(), ex);
                failureCallback.handleFailure(ex.toString());
            }
        }
    }
}
