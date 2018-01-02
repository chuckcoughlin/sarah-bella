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

import android.app.Activity;
import android.content.Context;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkInfo;
import android.util.Log;


import android.net.wifi.WifiManager;
import android.net.wifi.SupplicantState;
import android.net.wifi.WifiInfo;

import ros.android.appmanager.SBRobotConnectionHandler;


/**
 * Threaded network checker. Looks at the array of networks available
 * and connects to the correct one. Performs the function of the
 * former WiFiChecker.
 */
public class NetworkChecker {
  private final static String CLSS = "NetworkChecker";
  private final SBRobotConnectionHandler handler ;
  private final ConnectivityManager connectivityManager;
  private CheckerThread checkerThread;

  /**
   * Constructor. Should not take any time.
   */
  public NetworkChecker(Activity activity,SBRobotConnectionHandler h) {
    this.connectivityManager = (ConnectivityManager)activity.getApplicationContext().getSystemService(Context.CONNECTIVITY_SERVICE);
    this.handler = h;
  }

  public static boolean wifiValid(RobotId robotId, WifiManager wifiManager) {
    WifiInfo wifiInfo = wifiManager.getConnectionInfo();
    if (robotId.getWifi() == null) { //Does not matter what wifi network, always valid.
      return true;
    }
    if (wifiManager.isWifiEnabled()) {
      if (wifiInfo != null) {
        Log.d("NetworkChecker", "WiFi Info: " + wifiInfo.toString() + " IP " + wifiInfo.getIpAddress());
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

  /**
   * Start the checker thread with the given robotId. If the thread is
   * already running, kill it first and then start anew. Returns immediately.
   * For now we only check Wifi.
   */
  public void beginChecking(RobotId robotId) {
    stopChecking();
    //If there's no wifi tag in the robot id, skip this step
    if (robotId.getWifi() == null) {
      handler.handleNetworkError("No WiFi definition in robotId");
      return;
    }

    checkerThread = new CheckerThread(robotId,connectivityManager);
    checkerThread.start();
  }

  /** Stop the checker thread. */
  public void stopChecking() {
    if (checkerThread != null && checkerThread.isAlive()) {
      checkerThread.interrupt();
    }
  }

  /**
   * Loop over all connected networks, looking for an appropriate one.
   * If found, make it the current network.
   */
  private class CheckerThread extends Thread {

    private RobotId robotId;
    private ConnectivityManager connectivityManager;

    public CheckerThread(RobotId robotId, ConnectivityManager mgr) {
      this.robotId = robotId;
      this.connectivityManager = mgr;

      setDaemon(true);

      // don't require callers to explicitly kill all the old checker threads.
      setUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() {
        @Override
        public void uncaughtException(Thread thread, Throwable ex) {
          handler.handleNetworkError("Exception: " + ex.getMessage());
        }
      });
    }

    @Override
    public void run() {
      Network active = null;
      Network[] networks = connectivityManager.getAllNetworks();
      for (Network net : networks) {
        NetworkInfo info = connectivityManager.getNetworkInfo(net);
        String msg = String.format("Found %s connected %s", info.getTypeName(), info.getState());
        Log.i(CLSS, msg);
        if (info.getType() == ConnectivityManager.TYPE_WIFI && info.isConnected()) {
          active = net;
        }
      }
      if (active == null) {
        String msg = String.format("No connnected WiFi network.");
        Log.e(CLSS, msg);
        handler.handleNetworkError(msg);
      }
      else {
        connectivityManager.bindProcessToNetwork(active);
        handler.receiveWifiConnection();
      }
    }
  }
}
