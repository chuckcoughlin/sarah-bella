/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *   This class functions as the ros.android.util.MasterChooser
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.ros;

import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteStatement;
import android.os.Handler;
import android.util.Log;

import org.ros.exception.RosException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Date;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;

import chuckcoughlin.sb.assistant.db.SBDbManager;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

// From rosjava

/**
 * Encapsulate the description of the robot to which we are connected. his class is designed to
 * accommodate the existence of only a single robot. Maintain the connection state.
 * Since we access from multiple fragments, make this a singleton class to avoid repeated
 * allocations. It is created and shutdown in the MainActivity.
 */
public class SBRosManager {
    private final static String CLSS = "SBRosManager";

    private static volatile SBRosManager instance = null;
    private String bluetoothError;
    private String wifiError;
    private boolean networkConnected;
    private Thread nodeThread;
    private Handler uiThreadHandler = new Handler();
    private RobotDescription robot = null;
    private NodeConfiguration configuration = null;

    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * Initialize instance variables.
     */
    private SBRosManager() {
        //Prevent form the reflection api.
        if (instance != null) {
            throw new RuntimeException("Attempt to instantiate SBRosManager singleton via reflection");
        }
        this.robot = null;
        this.bluetoothError = null;
        this.wifiError = null;
        this.networkConnected = false;
    }

    /**
     * Use this method to access the singleton object.
     *
     * @return the Singleton instance
     */
    public static SBRosManager getInstance() {
        // Use the application context, which will ensure that you
        // don't accidentally leak an Activity's context.
        if (instance == null) {
            synchronized (SBRosManager.class) {
                instance = new SBRosManager();
            }
        }
        return instance;
    }

    public RobotDescription getRobot() {
        return this.robot;
    }

    public void setRobot(RobotDescription desc) {
        this.robot = desc;
    }

    public String getConnectionStatus() {
        if (robot != null) return robot.getConnectionStatus();
        return RobotDescription.CONNECTION_STATUS_UNCONNECTED;
    }

    public boolean hasBluetoothError() {
        return bluetoothError != null;
    }

    public void setBluetoothError(String reason) {
        this.bluetoothError = reason;
        this.robot = null;
        this.networkConnected = false;
    }

    public void setConnectionStatus(String status) {
        if (robot != null) robot.setConnectionStatus(status);
    }

    // Set the Bluetooth device name of the current robot and update database.
    public void setDeviceName(String name) {
        if (this.robot != null) {
            this.robot.getRobotId().setDeviceName(name);
        }
    }

    public void setNetworkConnected(boolean flag) {
        if (flag) {
            bluetoothError = null;
            wifiError = null;
        } else {
            this.robot = null;
        }
        networkConnected = flag;
    }

    // Set the SSID of the current robot and update database.
    public void setSSID(String ssid) {
        if (this.robot != null) {
            this.robot.getRobotId().setSSID(ssid);
        }
    }

    public void setWifiError(String reason) {
        this.wifiError = reason;
        this.robot = null;
        this.networkConnected = false;
    }


    private void shutdown() {
    }

    /**
     * Called when main activity is destroyed. Clean up any resources.
     * To use again requires re-initialization.
     */
    public static void destroy() {
        if (instance != null) {
            synchronized (SBRosApplicationManager.class) {
                instance.shutdown();
                instance = null;
            }
        }
    }
}