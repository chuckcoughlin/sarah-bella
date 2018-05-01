/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *   This class functions as the ros.android.util.MasterChooser
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.ros;

import android.os.Handler;

import org.ros.node.NodeConfiguration;

import ros.android.util.RobotDescription;

// From rosjava

/**
 * Encapsulate the description of the robot to which we are connected. This class is designed to
 * accommodate the existence of only a single robot. Maintain the connection state.
 * Since we access from multiple fragments, make this a singleton class to avoid repeated
 * allocations. It is created and shutdown in the MainActivity.
 *
 * Keep track of the state of the remote RosCore.
 */
public class SBRobotManager {
    private final static String CLSS = "SBRobotManager";
    // Network Connection Status
    public static final String STATE_UNCONNECTED="UNCONNECTED";    // We have no knowledge re internal state of robot
    public static final String STATE_CONNECTING = "CONNECTING";    // We are attempting to establish a connection
    public static final String STATE_STARTING   = "STARTING";      // RosCore is not "yet" available (we did a restart?)
    public static final String STATE_RUNNING    = "RUNNING";       // RosCore is communicating
    public static final String STATE_UNAVAILABLE= "UNAVAILABLE";   // Network error, no communication possible

    private static volatile SBRobotManager instance = null;
    private String bluetoothError;
    private String wifiError;
    private String connectionStatus;
    private Thread nodeThread;
    private Handler uiThreadHandler = new Handler();
    private RobotDescription robot = null;
    private NodeConfiguration configuration = null;

    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * Initialize instance variables.
     */
    private SBRobotManager() {
        //Prevent form the reflection api.
        if (instance != null) {
            throw new RuntimeException("Attempt to instantiate SBRobotManager singleton via reflection");
        }
        this.robot = null;
        this.bluetoothError = null;
        this.wifiError = null;
        this.connectionStatus = STATE_UNCONNECTED;
    }

    /**
     * Use this method to access the singleton object.
     *
     * @return the Singleton instance
     */
    public static SBRobotManager getInstance() {
        // Use the application context, which will ensure that you
        // don't accidentally leak an Activity's context.
        if (instance == null) {
            synchronized (SBRobotManager.class) {
                instance = new SBRobotManager();
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

    public String getConnectionStatus() { return this.connectionStatus; }

    public boolean hasBluetoothError() {
        return bluetoothError != null;
    }

    public void setBluetoothError(String reason) {
        this.bluetoothError = reason;
        this.robot = null;
        this.connectionStatus = STATE_UNCONNECTED;
    }

    public void setConnectionStatus(String status) { this.connectionStatus = status; }

    // Set the Bluetooth device name of the current robot and update database.
    public void setDeviceName(String name) {
        if (this.robot != null) {
            this.robot.getRobotId().setDeviceName(name);
        }
    }

    // A network connection is only the first step. We still have ROS.
    public void setNetworkConnected(boolean flag) {
        if (flag) {
            bluetoothError = null;
            wifiError = null;
            setConnectionStatus(STATE_CONNECTING);
        }
        else {
            this.robot = null;
            setConnectionStatus(STATE_UNCONNECTED);
        }
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
        setConnectionStatus(STATE_UNAVAILABLE);
    }


    private void shutdown() {
    }

    /**
     * Called when main activity is destroyed. Clean up any resources.
     * To use again requires re-initialization.
     */
    public static void destroy() {
        if (instance != null) {
            synchronized (SBApplicationManager.class) {
                instance.shutdown();
                instance = null;
            }
        }
    }
}