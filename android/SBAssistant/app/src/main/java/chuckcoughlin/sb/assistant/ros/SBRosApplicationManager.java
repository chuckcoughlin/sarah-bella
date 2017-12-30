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

import app_manager.App;
import app_manager.StatusCodes;
import chuckcoughlin.sb.assistant.db.SBDbManager;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

// From rosjava

/**
 * Since we access from multiple fragments, make this a singleton class to avoid repeated
 * allocations. This class makes the list of applications for the current robot available
 * to be started and stopped.
 *
 * The Apps list is in the database. We keep the current application and its status locally.
 */
public class SBRosApplicationManager {
    private final static String CLSS = "SBRosManager";
    // Keys for columns in result map. Somewhere there's a 6 char limit.
    public static final String URI_VALUE = "URI";
    public static final String NAME_VALUE = "NAME";
    public static final String DISPLAY_VALUE = "DISPLY";
    public static final String STATUS_VALUE = "STATUS";

    private static SBRosApplicationManager instance = null;
    private final SBDbManager dbManager;
    private final SBRosManager rosManager;
    private final Context context;
    private Thread nodeThread;
    private Handler uiThreadHandler = new Handler();
    private String currentApplication;
    private int status;     // Status of current application


    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * @param context
     */
    private SBRosApplicationManager(Context context) {
        this.context = context.getApplicationContext();
        this.dbManager = SBDbManager.getInstance();
        this.rosManager = SBRosManager.getInstance();
        this.currentApplication = null;     // Name
        this.status = StatusCodes.NOT_RUNNING;
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

    /**
     * Insert a new row into the application table. An attempt to add a duplicate
     * application is ignored.
     * NOTE: Only the status is updatable.
     * @param app application
     */
    public void addApplication(App app) {
        RobotDescription robot = rosManager.getRobot();
        if (robot == null) return;
        RobotId id = robot.getRobotId();

        StringBuilder sql = new StringBuilder("INSERT INTO RobotApplications(masterUri,appName,displayName");
        sql.append(" VALUES(?,?,?)");
        SQLiteDatabase db = dbManager.getWritableDatabase();
        SQLiteStatement stmt = db.compileStatement(sql.toString());
        stmt.bindString(1, id.getMasterUri());
        stmt.bindString(2, app.getName());
        stmt.bindString(3, app.getDisplayName());
        try {
            stmt.executeInsert();
        } catch (SQLException sqle) {
            Log.w(CLSS, String.format("Attempt to add application %s:%s failed (%s)", id.getMasterUri(), app.getName(), sqle.getLocalizedMessage()));
        } finally {
            stmt.close();
        }
    }

    public void clearApplications() {
        RobotDescription robot = rosManager.getRobot();
        if (robot == null) return;
        String uri = robot.getRobotId().getMasterUri();
        String sql = String.format("DELETE FROM RobotApplications WHERE masterUri = '%s'", uri);
        dbManager.execSQL(sql);
    }

    public List<App> getApplications() {
        List<App> apps = new ArrayList<>();
        RobotDescription robot = rosManager.getRobot();
        if (robot == null) return apps;

        String uri = robot.getRobotId().getMasterUri();
        SQLiteDatabase db = dbManager.getReadableDatabase();

        StringBuilder sql = new StringBuilder(
                "SELECT masterUri,appName,displayName");
        sql.append(" FROM RobotApplications");
        sql.append(" WHERE masterUri = ?");
        sql.append(" ORDER BY appName");
        String[] args = new String[]{uri};

        Cursor cursor = db.rawQuery(sql.toString(), args);
        cursor.moveToFirst();
        while (!cursor.isAfterLast()) {
            Map<String, Object> map = new HashMap<>();
            map.put(URI_VALUE, cursor.getString(1));
            map.put(NAME_VALUE, cursor.getString(2));
            map.put(DISPLAY_VALUE, cursor.getString(3));

            RobotId id = new RobotId(map);
            // "nodeless" message creation
            NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
            MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
            App app = messageFactory.newFromType(App._TYPE);
            app.setName(cursor.getString(2));
            app.setDisplayName(cursor.getString(3));
            apps.add(app);
            cursor.moveToNext();
        }
        cursor.close();
        return apps;
    }

    public String getCurrentApplication() { return this.currentApplication; }
    public void setCurrentApplication(String name) { this.currentApplication=name; }
    public int getStatus() { return this.status; }
    public String getStatusAsString() {
        switch (this.status) {
            case StatusCodes.NOT_RUNNING:
                return "Not running";
            default:
                return String.format("Unknown status(%d)",this.status);
        }
    }
    public void setCurrentStatus(int s) { this.status=s; }
    /***
     * Replace the list of apps with the list specified.
     * @param apps a list of applications defined on the current robot
     */
    public void setApplications(List<App> apps) {
        clearApplications(); // Start clean.
        for (App app : apps) {
            addApplication(app);
        }
    }

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

    /**
     * Set the current status for an application. Initial status is NOT_RUNNING.
     * @param appName application name
     * @param status new status
     */
    public void setStatus(String appName, int status) {
        RobotDescription robot = rosManager.getRobot();
        if (robot == null) return;
        RobotId id = robot.getRobotId();
        String uri = robot.getRobotId().getMasterUri();
        StringBuilder sql = new StringBuilder("UPDATE RobotApplications SET status = ?");
        sql.append(" WHERE masterUri = ?");
        sql.append("   AND appName=?");
        String[] args = {String.valueOf(status), uri, appName};
        SQLiteDatabase db = dbManager.getWritableDatabase();
        SQLiteStatement stmt = db.compileStatement(sql.toString());
        stmt.bindAllArgsAsStrings(args);
        try {
            stmt.executeUpdateDelete();
        } catch (SQLException sqle) {
            Log.w(CLSS, String.format("Attempt to update application status %s:%s failed (%s)", uri, appName, sqle.getLocalizedMessage()));
        } finally {
            stmt.close();
        }
    }
}
