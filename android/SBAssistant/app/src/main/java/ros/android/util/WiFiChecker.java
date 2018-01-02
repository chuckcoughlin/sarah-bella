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

package ros.android.util;

import android.util.Log;


import android.net.wifi.WifiManager;
import android.net.wifi.WifiConfiguration;
import android.net.wifi.SupplicantState;
import android.net.wifi.WifiInfo;
import android.content.Context;

import ros.android.appmanager.SBRobotConnectionHandler;


/**
 * Threaded WiFi checker. Checks and tests if the WiFi is configured properly and if not, connects to the correct network.
 *
 * @author pratkanis@willowgarage.com
 */
public class WiFiChecker {
  private final static String CLSS = "WifiChecker";

  private CheckerThread checkerThread;
  private final SBRobotConnectionHandler handler ;

  /** Constructor. Should not take any time. */
  public WiFiChecker(SBRobotConnectionHandler h) {
    this.handler = h;
  }

  /**
   * Start the checker thread with the given robotId. If the thread is
   * already running, kill it first and then start anew. Returns immediately.
   */
  public void beginChecking(RobotId robotId, WifiManager manager) {
    stopChecking();
    //If there's no wifi tag in the robot id, skip this step
    if (robotId.getWifi() == null) {
      handler.handleWifiError("WifiChecker: No WiFi definition in robotId");
      return;
    }

    checkerThread = new CheckerThread(robotId, manager);
    checkerThread.start();
  }

  /** Stop the checker thread. */
  public void stopChecking() {
    if (checkerThread != null && checkerThread.isAlive()) {
      checkerThread.interrupt();
    }
  }

  public static boolean wifiValid(RobotId robotId, WifiManager wifiManager) {
    WifiInfo wifiInfo = wifiManager.getConnectionInfo();
    if (robotId.getWifi() == null) { //Does not matter what wifi network, always valid.
      return true;
    }
    if (wifiManager.isWifiEnabled()) {
      if (wifiInfo != null) {
        Log.d("WiFiChecker", "WiFi Info: " + wifiInfo.toString() + " IP " + wifiInfo.getIpAddress());
        if (wifiInfo.getSSID() != null && wifiInfo.getIpAddress() != 0
            && wifiInfo.getSupplicantState() == SupplicantState.COMPLETED) {
          if (wifiInfo.getSSID().equals(robotId.getWifi())) {
            return true;
          }
        }
      }
    }
    return false;
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
          handler.handleWifiError("exception: " + ex.getMessage());
        }
      });
    }

    private boolean wifiValid() {
      return WiFiChecker.wifiValid(robotId, wifiManager);
    }

    @Override
    public void run() {
      try {
        if (wifiValid()) {
          handler.receiveWifiConnection();
        }
        else if (handler.handleWifiReconnection(wifiManager.getConnectionInfo().getSSID(), robotId.getWifi())) {
          Log.d("WiFiChecker", "Disable networking");
          wifiManager.setWifiEnabled(false);
          int i = 0;
          while (i < 30 && wifiManager.isWifiEnabled()) {
            Log.d("WiFiChecker", "Waiting for WiFi enable");
            Thread.sleep(1000L);
            i++;
          }
          if (wifiManager.isWifiEnabled()) {
            handler.handleWifiError("Un-able to shutdown WiFi");
            return;  
          }
          Log.d("WiFiChecker", "Wait for networking");
          wifiManager.setWifiEnabled(true);
          while (i < 30 && !wifiManager.isWifiEnabled()) {
            Log.d("WiFiChecker", "Waiting for WiFi enable");
            Thread.sleep(1000L);
            i++;
          }
          if (!wifiManager.isWifiEnabled()) {
            handler.handleWifiError("Un-able to connect to WiFi");
            return;  
          }

          int n = -1;
          int priority = -1;
          WifiConfiguration wc = null;
          String SSID = "\"" + robotId.getWifi() + "\"";
          for (WifiConfiguration test : wifiManager.getConfiguredNetworks()) {
            Log.d("WiFiChecker", "WIFI " + test.toString());
            if (test.priority > priority) {
              priority = test.priority;
            }
            if (test.SSID.equals(SSID)) {
              n = test.networkId;
              wc = test;
            }
            test.status = WifiConfiguration.Status.DISABLED;
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
            Log.d("WiFiChecker", "WIFI Unknown");
            wc = new WifiConfiguration();
            wc.SSID = "\"" + robotId.getWifi() + "\"";
            if (robotId.getWifiPassword() != null) {
              wc.preSharedKey  = "\"" + robotId.getWifiPassword() + "\"";
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
            Log.d("WiFiChecker", "add Network returned " + n);
            if (n == -1) {
              handler.handleWifiError("Failed to configure WiFi");
            }
          }
          
          //Connect to the network
          boolean b = wifiManager.enableNetwork(n, true);
          Log.d("WiFiChecker", "enableNetwork returned " + b);
          if (b) {
            wifiManager.reconnect();
            Log.d("WiFiChecker", "Wait for wifi network");
            i = 0;
            while (i < 90 && !wifiValid()) {
              Log.d("WiFiChecker", "Waiting for network: " + i + " " + wifiManager.getWifiState());
              Thread.sleep(1000L);
              i++;
            }
            if (wifiValid()) {
              handler.receiveWifiConnection();
            } else {
              handler.handleWifiError("WiFi connection timed out");
            }
          }
        }
        else {
          handler.handleWifiError("Wrong WiFi network");
        }
      }
      catch (Throwable ex) {
        Log.e("RosAndroid", "Exception while searching for WiFi for "
              + robotId.getWifi(), ex);
        handler.handleWifiError(ex.toString());
      }
    }
  }
}
