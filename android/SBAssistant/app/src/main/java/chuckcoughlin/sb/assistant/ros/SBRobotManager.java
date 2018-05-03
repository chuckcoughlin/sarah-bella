/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *   This class functions as the ros.android.util.MasterChooser
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.ros;

import ros.android.util.RobotDescription;
import ros.android.util.RobotId;


/**
 * Maintain the state of network connection to the robot. When connected, this class encapsulates
 * a description of the robot. We accommodate the existence of only a single robot.
 * Since we access from multiple fragments, make this a singleton class to avoid repeated
 * allocations. It is created and shutdown in the MainActivity.
 *
 * Keep track of the state of the remote RosCore.
 * These are the states:
 *   UNCONNECTED          - we have yet to attempt a network connection
 *   BLUETOOTH_CONNECTING - we are attempting a Bluetooth connection
 *   WIFI_CONNECTING      - we are attempting a Wifi connection (bluetooth failed)
 *   STARTING_LOCAL       - start the local RosCore (on tablet)
 *   OFFLINE              - local node is running, but the remote is not
 *   STARTING_REMOTE      - start the remote node (already running unless we've shut it down)
 *   ONLINE               - local and remote ROS nodes are communicating
 *   UNAVAILABLE          - network error, cannot communicate
 *
 *   Note: The RobotDescription is valid only during the "ONLINE" state, but we hold it as
 *   a repository for misc network and other configuration parameters.
 */
public class SBRobotManager {
    private final static String CLSS = "SBRobotManager";
    // Network Connection States
    public static final String STATE_UNCONNECTED          = "UNCONNECTED";
    public static final String STATE_BLUETOOTH_CONNECTING = "BLUETOOTH_CONNECTING";
    public static final String STATE_WIFI_CONNECTING      = "WIFI_CONNECTING";
    public static final String STATE_STARTING_LOCAL       = "STARTING_LOCAL";
    public static final String STATE_OFFLINE              = "OFFLINE";
    public static final String STATE_STARTING_REMOTE      = "STARTING_REMOTE";
    public static final String STATE_ONLINE               = "ONLINE";
    public static final String STATE_UNAVAILABLE          = "UNAVAILABLE";

    private static volatile SBRobotManager instance = null;
    private String connectionState;
    private final RobotDescription robot;


    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * Initialize instance variables.
     */
    private SBRobotManager() {
        //Prevent form the reflection api.
        if (instance != null) {
            throw new RuntimeException("Attempt to instantiate SBRobotManager singleton via reflection");
        }
        this.robot = RobotDescription.createUnknown(new RobotId());
        this.connectionState = STATE_UNCONNECTED;
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

    public RobotDescription getRobot() { return this.robot; }

    public String getConnectionState() { return this.connectionState; }
    public void setConnectionState(String status) { this.connectionState = status; }

    // Set the Bluetooth paired-device name
    public void setDeviceName(String name) {
        this.robot.getRobotId().setDeviceName(name);
    }
    // Set the master (remote) URL, string form
    public void setMasterURI(String uriString) {
        this.robot.getRobotId().setMasterUri(uriString);
    }
    // Set the Wifi password
    public void setWifiPassword(String pw)     { this.robot.getRobotId().setWifiPassword(pw); }
    // Set the Wifi network name
    public void setSSID(String ssid) {
        this.robot.getRobotId().setSSID(ssid);
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