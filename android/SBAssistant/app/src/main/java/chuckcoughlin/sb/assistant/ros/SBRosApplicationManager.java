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
 * allocations. This class handles the publications and subscriptions corresponding to the
 * application on the current robot. Fragments subscribe here and messages received from the
 * robot are parcelled out to the various listeners.
 *
 * The Apps list is in the database and is hard-coded. It represents the capabilities
 * of the robot we connect to. We keep the current application and its status locally.
 */
public class SBRosApplicationManager {
    private final static String CLSS = "SBRosManager";

    private static SBRosApplicationManager instance = null;
    private final SBDbManager dbManager;
    private final SBRosManager rosManager;
    private final Context context;
    private Thread nodeThread;
    private Handler uiThreadHandler = new Handler();
    private RobotApplication application;


    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * @param context
     */
    private SBRosApplicationManager(Context context) {
        this.context = context.getApplicationContext();
        this.dbManager = SBDbManager.getInstance();
        this.rosManager = SBRosManager.getInstance();
        this.application = null;
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

    public RobotApplication getApplication() { return this.application; }


    /**
     * Specify the name of a the current application. From this we query the
     * database to get the full list of publishers and subscribers.
     * @param name name of the application
     */
    public void setApplicationFromName(String name) {
        this.application = null;
        RobotDescription robot = rosManager.getRobot();
        if (robot != null) {
            SQLiteDatabase db = dbManager.getReadableDatabase();

            StringBuilder sql = new StringBuilder(
                    "SELECT appName,description");
            sql.append(" FROM RobotApplications");
            sql.append(" ORDER BY appName");
            String[] args = new String[]{};

            Cursor cursor = db.rawQuery(sql.toString(), args);
            cursor.moveToFirst();
            if (!cursor.isAfterLast()) {
                Map<String, Object> map = new HashMap<>();

                application = new RobotApplication(cursor.getString(0),cursor.getString(1));
            }
            cursor.close();
            //TODO: Add publishers and subscribers

        };
    }

}
