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

import chuckcoughlin.sb.assistant.ros.SBRosManager;
import ros.android.util.RobotId;

public class WifiChecker {
    private final static String CLSS = "WifiChecker";
    private CheckerThread checkerThread;
    private SBRobotConnectionHandler handler;
    private static String wifiError = "";

    public WifiChecker(SBRobotConnectionHandler handler) {
        this.handler = handler;
    }

    // An empty string returned implies success, else an error message
    public static boolean wifiValid(RobotId robotId, WifiManager wifiManager) {
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
            Log.d(CLSS, String.format("%s:WiFi OK IP %x, SSID: %s %s(hidden)",wifiInfo.getIpAddress(),wifiInfo.getSSID(),wifiInfo.getHiddenSSID()));
        }
        return result;
    }

    public void beginChecking(RobotId robotId, WifiManager manager) {
        stopChecking();
        //If there's no wifi tag in the robot id, skip this step
        if (robotId.getSSID() == null) {
            handler.receiveWifiConnection();
            return;
        }
        checkerThread = new CheckerThread(robotId, manager);
        checkerThread.start();
    }

    public void stopChecking() {
        if (checkerThread != null && checkerThread.isAlive()) {
            checkerThread.interrupt();
        }
    }

    private class CheckerThread extends Thread {
        private RobotId robotId;
        private WifiManager wifiManager;

        public CheckerThread(RobotId robotId, WifiManager wifi) {
            this.robotId = robotId;
            this.wifiManager = wifi;
            setDaemon(true);
            // don't require callers to explicitly kill all the old checker threads.
            setUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() {
                @Override
                public void uncaughtException(Thread thread, Throwable ex) {
                    handler.handleNetworkError("exception: " + ex.getMessage());
                }
            });
        }

        // If valid set the SSID, else set the error message
        private boolean wifiValid() {
            return  WifiChecker.wifiValid(robotId, wifiManager);
        }

        @Override
        public void run() {
            try {
                if (wifiValid()) {
                    WifiInfo info = wifiManager.getConnectionInfo();
                    String ssid = info.getSSID().replaceAll("\"","");
                    SBRosManager.getInstance().setSSID(ssid);
                    handler.receiveWifiConnection();
                }
                else if (handler.handleWifiReconnection(wifiManager.getConnectionInfo().getSSID(), robotId.getSSID())) {
                    Log.i(CLSS, "Wait for networking");
                    wifiManager.setWifiEnabled(true);
                    int i = 0;
                    while (i < 30 && !wifiManager.isWifiEnabled()) {
                        Log.i(CLSS, "Waiting for WiFi enable");

                        i++;
                    }
                    if (!wifiManager.isWifiEnabled()) {
                        handler.handleNetworkError("Un-able to connect to WiFi");
                        return;
                    }
                    int n = -1;
                    int priority = -1;
                    WifiConfiguration wc = null;
                    String SSID = "\"" + robotId.getSSID() + "\"";
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

                    //Add new network.
                    if (n == -1) {
                        Log.i(CLSS, "WIFI Unknown");
                        wc = new WifiConfiguration();
                        wc.SSID = "\"" + robotId.getSSID() + "\"";
                        if (robotId.getWifiPassword() != null) {
                            wc.preSharedKey = "\"" + robotId.getWifiPassword() + "\"";
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

                            i++;
                        }
                        if (wifiValid()) {
                            handler.receiveWifiConnection();
                        }
                        else {
                            handler.handleNetworkError("WiFi connection timed out");
                        }
                    }
                }
                else {
                    handler.handleNetworkError("Wrong WiFi network");
                }
            }
            catch (Throwable ex) {
                Log.e("RosAndroid", "Exception while searching for WiFi for "
                        + robotId.getSSID(), ex);
                handler.handleNetworkError(ex.getLocalizedMessage());
            }
        }
    }
}


