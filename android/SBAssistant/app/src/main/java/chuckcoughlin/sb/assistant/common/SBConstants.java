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

    public static final String DB_NAME = "SBAssistant.db";
    public static final int DB_VERSION = 2;
    // These are the Settings table columns
    public static final String SETTINGS_NAME  = "name";
    public static final String SETTINGS_VALUE = "value";

    // These are the parameter names in the Settings table
    public static final String ROS_MASTER_URI="ROS Master URI";
    public static final String ROS_GATEWAY="Gateway";
    public static final String ROS_SSID="Wifi Network Name";
    public static final String ROS_WIFIPWD="Wifi Password";
    public static final String ROS_USER="Robot Username";
    public static final String ROS_USER_PASSWORD="Robot Password";

    // These are the default values for the settings
    public static final String DEFAULT_ROS_MASTER_URI="http://xxx.xxx.xxx.xxx:11311";
    public static final String DEFAULT_ROS_GATEWAY="192.168.0.1";
    public static final String DEFAULT_ROS_SSID="network";
    public static final String DEFAULT_ROS_WIFIPWD="";
    public static final String DEFAULT_ROS_USER="username";
    public static final String DEFAULT_ROS_USER_PASSWORD="";

    // Application Names
    public static final String APPLICATION_SYSTEM="System";
    // Platform Types
    public static final String PLATFORM_LINUX="Linux";
    public static final String PLATFORM_RASPBERRYPI="RaspberryPy";

    // Dialog transaction key
    public static final String DIALOG_TRANSACTION_KEY = "dialog";

    // Dialog results buttons
    public static final String DIALOG_BUTTON_ADD = "add";
    public static final String DIALOG_BUTTTON_CANCEL = "cancel";
}