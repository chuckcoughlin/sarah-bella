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
 * See: http://docs.ros.org/hydro/api/android_apps/html/WifiChecker_8java_source.html
 */

package ros.android.appmanager;

import android.util.Log;
import android.net.wifi.WifiManager;
import android.net.wifi.WifiConfiguration;
import android.net.wifi.SupplicantState;
import android.net.wifi.WifiInfo;

import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBRobotManager;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

public class WifiChecker {
    private final static String CLSS = "WifiChecker";
    private CheckerThread checkerThread;
    private SBRobotConnectionHandler handler;
    private static String wifiError = "";
    private boolean threadRunning;

    public WifiChecker(SBRobotConnectionHandler handler) {
        this.threadRunning = false;
        this.handler = handler;
    }

    // An empty string returned implies success, else an error message
    public static boolean wifiValid(WifiManager wifiManager) {
        WifiInfo wifiInfo = wifiManager.getConnectionInfo();
        boolean result = false;
        if(!wifiManager.isWifiEnabled()) {
            wifiError = "WiFi network is not enabled";
        }
        else if (wifiInfo == null) {
            wifiError = "No WiFi network information available.";
        }
        else if (wifiInfo.getSSID() == null || wifiInfo.getIpAddress() == 0 ) {
            wifiError = "Null WiFi network address.";
        }
        else if( wifiInfo.getSupplicantState()!= SupplicantState.COMPLETED ){
            wifiError = String.format("WiFi supplicant state (%d) is not completed",wifiInfo.getSupplicantState());

        }
        else {
            result = true;
            Log.d(CLSS, String.format("WiFi OK IP %x, SSID: %s %s(hidden)",wifiInfo.getIpAddress(),wifiInfo.getSSID(),wifiInfo.getHiddenSSID()));
        }
        return result;
    }

    public void beginChecking(RobotDescription robot, WifiManager manager) {
        if( this.threadRunning ) {
            Log.i(CLSS, "check already in progress ...");
            return;
        }

        checkerThread = new CheckerThread(robot, manager);
        checkerThread.start();
    }

    public void stopChecking() {
        if (checkerThread != null && checkerThread.isAlive()) {
            checkerThread.interrupt();
        }
    }

    private class CheckerThread extends Thread {
        private RobotDescription robot;
        private WifiManager wifiManager;

        public CheckerThread(RobotDescription rbt, WifiManager wifi) {
            if( threadRunning ) {
                Log.i(CLSS, "check already in progress ...");
                return;
            }
            this.robot = rbt;
            this.wifiManager = wifi;
            setDaemon(true);
            // don't require callers to explicitly kill all the old checker threads.
            setUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() {
                @Override
                public void uncaughtException(Thread thread, Throwable ex) {
                    Log.e(CLSS, String.format("Uncaught exception checking WiFi Connection: %s",ex.getLocalizedMessage()),ex);
                    handler.handleNetworkError(String.format("Uncaught exception (%s)",ex.getLocalizedMessage()));
                    threadRunning = false;
                }
            });
        }

        private boolean wifiValid() {
            return  WifiChecker.wifiValid(wifiManager);
        }

        @Override
        public void run() {
            threadRunning = true;
            try {
                if (wifiValid()) {
                    WifiInfo info = wifiManager.getConnectionInfo();
                    String ssid = info.getSSID().replaceAll("\"","");
                    SBRobotManager.getInstance().setSSID(ssid);
                    handler.receiveNetworkConnection();;
                }
                // Try to reconnect
                else {
                    Log.i(CLSS, "Waiting for networking");
                    wifiManager.setWifiEnabled(true);
                    int i = 0;
                    while (i < 30 && !wifiManager.isWifiEnabled()) {
                        Log.i(CLSS, "Waiting for WiFi enable");
                        Thread.sleep(100);
                        if (!wifiManager.isWifiEnabled()) break;
                        i++;
                    }
                    if (!wifiManager.isWifiEnabled()) {
                        handler.handleNetworkError("Un-able to connect to WiFi");
                        return;
                    }
                    int n = -1;
                    int priority = -1;
                    WifiConfiguration wc = null;
                    String SSID = "\"" + robot.getRobotId().getSSID() + "\"";
                    for (WifiConfiguration test : wifiManager.getConfiguredNetworks()) {
                        Log.i(CLSS, "WIFI " + test.toString());
                        if (test.priority > priority) {
                            priority = test.priority;
                        }
                        if (test.SSID.equals(SSID)) {
                            n = test.networkId;
                            wc = test;
                        }
                    }
                    if (wc != null) {
                        if (wc.priority != priority) {
                            wc.priority = priority + 1;
                        }
                        wc.status = WifiConfiguration.Status.DISABLED;
                        wifiManager.updateNetwork(wc);
                    }

                    // In case of no network, add a new network based on configured settings.
                    if (n == -1) {
                        Log.i(CLSS, "WIFI Unknown");
                        wc = new WifiConfiguration();
                        wc.SSID = "\"" + robot.getRobotId().getSSID() + "\"";
                        if (robot.getRobotId().getWifiPassword() != null) {
                            wc.preSharedKey = "\"" + robot.getRobotId().getWifiPassword() + "\"";
                        } else {
                            wc.preSharedKey = null;
                        }
                        wc.hiddenSSID = true;
                        wc.status = WifiConfiguration.Status.DISABLED;
                        wc.allowedAuthAlgorithms.set(WifiConfiguration.AuthAlgorithm.LEAP);
                        wc.allowedAuthAlgorithms.set(WifiConfiguration.AuthAlgorithm.OPEN);
                        wc.allowedAuthAlgorithms.set(WifiConfiguration.AuthAlgorithm.SHARED);
                        wc.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.CCMP);
                        wc.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.TKIP);
                        wc.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.WEP104);
                        wc.allowedGroupCiphers.set(WifiConfiguration.GroupCipher.WEP40);
                        wc.allowedKeyManagement.set(WifiConfiguration.KeyMgmt.NONE);
                        wc.allowedKeyManagement.set(WifiConfiguration.KeyMgmt.IEEE8021X);
                        wc.allowedKeyManagement.set(WifiConfiguration.KeyMgmt.WPA_EAP);
                        wc.allowedKeyManagement.set(WifiConfiguration.KeyMgmt.WPA_PSK);
                        wc.allowedPairwiseCiphers.set(WifiConfiguration.PairwiseCipher.TKIP);
                        wc.allowedPairwiseCiphers.set(WifiConfiguration.PairwiseCipher.CCMP);
                        wc.allowedProtocols.set(WifiConfiguration.Protocol.RSN);
                        wc.allowedProtocols.set(WifiConfiguration.Protocol.WPA);

                        n = wifiManager.addNetwork(wc);
                        Log.i(CLSS, "add Network returned " + n);
                        if (n == -1) {
                            handler.handleNetworkError("Failed to configure WiFi");
                        }
                    }

                    //Connect to the network
                    boolean b = wifiManager.enableNetwork(n, true);
                    Log.i("NetworkChecker", "enableNetwork returned " + b);
                    if (b) {
                        wifiManager.reconnect();
                        Log.i(CLSS, "Wait for wifi network");
                        i = 0;
                        while (i < 30 && !wifiValid()) {
                            Log.d(CLSS, "Waiting for network: " + i + " " + wifiManager.getWifiState());
                            Thread.sleep(500);
                            i++;
                            if (wifiValid()) {
                                handler.receiveNetworkConnection();
                                break;
                            }
                        }
                        if (!wifiValid()) {
                            handler.receiveNetworkConnection();
                        }
                        else {
                            handler.handleNetworkError("WiFi connection timed out");
                        }
                    }
                }
            }
            catch (Throwable ex) {
                Log.e(CLSS, "Exception while searching for WiFi for "
                        + robot.getRobotId().getSSID(), ex);
                handler.handleNetworkError(ex.getLocalizedMessage());
            }
            finally {
                threadRunning = false;
            }
        }
    }
}