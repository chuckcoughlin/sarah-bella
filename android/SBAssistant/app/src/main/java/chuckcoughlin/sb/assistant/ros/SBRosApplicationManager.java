/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * Re-create ROS App objects from the database.
 * (MIT License)
 */
package chuckcoughlin.sb.assistant.ros;

import android.content.Context;
import android.database.Cursor;
import android.database.SQLException;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteStatement;
import android.os.Handler;
import android.util.Log;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


import chuckcoughlin.sb.assistant.db.SBDbManager;
import ros.android.util.RobotApplication;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

// From rosjava

/**
 * Since we access from multiple fragments, make this a singleton class to avoid repeated
 * allocations. This class makes the list of applications for the current robot available
 * to be started and stopped.
 *
 * The Apps list is in the database and is hard-coded. It represents the capabilities
 * of the robot we connect to. We keep the current application and its status locally.
 */
public class SBRosApplicationManager {
    private final static String CLSS = "SBRosManager";
    // Keys for columns in result map. Somewhere there's a 6 char limit.
    public static final String NAME = "NAME";
    public static final String APPLICATION = "APP";
    public static final String STATUS_VALUE = "STATUS";

    private static SBRosApplicationManager instance = null;
    private final SBDbManager dbManager;
    private final SBRosManager rosManager;
    private final Context context;
    private Thread nodeThread;
    private Handler uiThreadHandler = new Handler();
    private String currentApplication;
    private String applicationStatus;     // Status of current application


    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * @param context
     */
    private SBRosApplicationManager(Context context) {
        this.context = context.getApplicationContext();
        this.dbManager = SBDbManager.getInstance();
        this.rosManager = SBRosManager.getInstance();
        this.currentApplication = null;     // Name
        this.applicationStatus = RobotApplication.APP_STATUS_NOT_RUNNING;
    }

    /**
     * Use this method in the initial activity. We need to assign the context.
     * @param context
     * @return the Singleton instance
     */
    public static synchronized SBRosApplicationManager initialize(Context context) {
        // Use the application context, which will ensure that you
        // don't accidentally leak an Activity's context.
        if (instance == null) {
            instance = new SBRosApplicationManager(context.getApplicationContext());
        }
        return instance;
    }

    /**
     * Use this method for all subsequent calls. We often don't have
     * a convenient context.
     * @return the Singleton instance.
     */
    public static synchronized SBRosApplicationManager getInstance() {
        return instance;
    }

    public List<RobotApplication> getApplications() {
        List<RobotApplication> apps = new ArrayList<>();
        RobotDescription robot = rosManager.getRobot();
        if (robot == null) return apps;

        SQLiteDatabase db = dbManager.getReadableDatabase();

        StringBuilder sql = new StringBuilder(
                "SELECT appName,description");
        sql.append(" FROM RobotApplications");
        sql.append(" ORDER BY appName");
        String[] args = new String[]{};

        Cursor cursor = db.rawQuery(sql.toString(), args);
        cursor.moveToFirst();
        while (!cursor.isAfterLast()) {
            Map<String, Object> map = new HashMap<>();

            String name = cursor.getString(1);
            RobotApplication app = new RobotApplication(cursor.getString(1),cursor.getString(2));
            apps.add(app);
            cursor.moveToNext();
        }
        cursor.close();
        //TODO: Add publishers and subscribers

        return apps;
    }

    public String getCurrentApplication() { return this.currentApplication; }
    public void setCurrentApplication(String name) { this.currentApplication=name; }
    public String getApplicationStatus() { return this.applicationStatus; }
    public void setCurrentStatus(String s) { this.applicationStatus=s; }

    /**
     * @return the number of applications defined for the current robot.
     */
    public int getApplicationCount() {
        RobotDescription robot = rosManager.getRobot();
        if (robot == null) return 0;

        SQLiteDatabase db = dbManager.getReadableDatabase();

        String uri = robot.getRobotId().getMasterUri();
        StringBuilder sql = new StringBuilder(
                "SELECT count(*)");
        sql.append(" FROM RobotApplications");
        sql.append(" WHERE masterUri = ?");
        String[] args = new String[]{uri};
        Cursor cursor = db.rawQuery(sql.toString(), args);
        int count = cursor.getCount();
        cursor.close();
        return count;
    }
}
