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
    public static final String ROS_HOSTNAME="ROS Hostname";

    // These are the default values for the settings
    public static final String DEFAULT_ROS_MASTER_URI="http://xxx.xxx.xxx:11311";
    public static final String DEFAULT_ROS_HOSTNAME="xxx.xxx.xxx";

    // Dialog transaction key
    public static final String DIALOG_TRANSACTION_KEY = "dialog";
    // Dialog results keys
    public static final String DIALOG_ERROR = "error";     // Text is error message
    public static final String DIALOG_RESULT = "result";   // Button that was pushed
    // Dialog results buttons
    public static final String DIALOG_RESULT_ADD = "add";
    public static final String DIALOG_RESULT_CANCEL = "cancel";
    // Various payloads
    public static final String DIALOG_ROBOT_DESCRIPTION = "robot";


}