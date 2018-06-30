/*
 * Copyright (C) 2017 Chuck Coughlin
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.common;

/**
 * This class contains static strings used throughout
 * the spplication.
 */
public class SBConstants {
    // Database configuration
    public static final String DB_NAME = "SBAssistant.db";
    public static final int DB_VERSION = 5;
    // These are the Settings table columns
    public static final String SETTINGS_NAME  = "name";
    public static final String SETTINGS_VALUE = "value";

    // These are the parameter names in the Settings table
    public static final String ROS_MASTER_URI="ROS Master URI";
    public static final String ROS_GATEWAY="Gateway";
    public static final String ROS_HOST="Host";
    public static final String ROS_PAIRED_DEVICE="Paired Device";
    public static final String ROS_SSID="Wifi Network Name";
    public static final String ROS_WIFIPWD="Wifi Password";
    public static final String ROS_USER="Robot Username";
    public static final String ROS_USER_PASSWORD="Robot Password";

    // These are the default values for the settings
    public static final String ROS_MASTER_URI_HINT="http://xxx.xxx.xxx.xxx:11311";
    public static final String ROS_GATEWAY_HINT="192.168.0.1";
    public static final String ROS_HOST_HINT="192.168.1.113";
    public static final String ROS_PAIRED_DEVICE_HINT="bluetooth device name";
    public static final String ROS_SSID_HINT="wifi SSID";
    public static final String ROS_WIFIPWD_HINT="wifi password";
    public static final String ROS_USER_HINT="robot sudoer";
    public static final String ROS_USER_PASSWORD_HINT="robot sudoer password";

    // Name of the main node on the tablet
    public static final String MAIN_NODE_NAME = "tablet_node_main";

    // Number of GPIO pins on Raspberry Pi
    public static final int GPIO_PIN_COUNT = 40;
    public static final String ROS_WIDTH_PARAM = "/robot/width";
    public static final double SB_ROBOT_CLOSEST_APPROACH = 0.2;   // ~m
    public static final double SB_ROBOT_WIDTH = 0.2;              // ~m

    // Number of log messages to store/display
    public static final int NUM_LOG_MESSAGES = 100;

    // Application Names - these must match parameters set in robot launch files
    public static final String APPLICATION_FOLLOW="follow";
    public static final String APPLICATION_HEADLAMP="headlamp";
    public static final String APPLICATION_PARK="park";
    public static final String APPLICATION_SYSTEM="system";
    public static final String APPLICATION_TELEOP="teleop";
    // Platform Types
    public static final String PLATFORM_LINUX="Linux";
    public static final String PLATFORM_RASPBERRYPI="RaspberryPy";
    // Behavior types (teleop fragment)
    public static final String ROS_BEHAVIOR_PARAM = "/robot/behavior";
    public static final String SB_BEHAVIOR_COME     = "come";
    public static final String SB_BEHAVIOR_FOLLOW   = "follow";
    public static final String SB_BEHAVIOR_JOYSTICK = "joystick";
    public static final String SB_BEHAVIOR_PARK     = "park";

    // Dialog transaction key
    public static final String DIALOG_TRANSACTION_KEY = "dialog";

    // Dialog results buttons
    public static final String DIALOG_BUTTON_ADD = "add";
    public static final String DIALOG_BUTTTON_CANCEL = "cancel";
}