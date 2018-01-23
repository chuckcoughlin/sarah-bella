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
 * Since we access from multiple fragments, make this a singleton class to avoid repeated
 * allocations. This class is designed to accommodate the existence of only a single
 * robot. It is responsible for making sure that internal robot object is in sync with the
 * database.
 */
public class SBRosManager {
    private final static String CLSS = "SBRosManager";

    private static SBRosManager instance = null;
    private final SBDbManager dbManager;
    private final Context context;
    private Thread nodeThread;
    private Handler uiThreadHandler = new Handler();
    private RobotDescription robot = null;
    private NodeConfiguration configuration = null;


    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * @param context
     */
    private SBRosManager(Context context) {
        this.context = context.getApplicationContext();
        this.dbManager = SBDbManager.getInstance();
        this.robot = initializeRobot();
    }

    /**
     * Use this method in the initial activity. We need to assign the context.
     * @param context
     * @return the Singleton instance
     */
    public static synchronized SBRosManager initialize(Context context) {
        // Use the application context, which will ensure that you
        // don't accidentally leak an Activity's context.
        if (instance == null) {
            instance = new SBRosManager(context.getApplicationContext());
        }
        return instance;
    }

    /**
     * Use this method for all subsequent calls. We often don't have
     * a convenient context.
     * @return the Singleton instance.
     */
    public static synchronized SBRosManager getInstance() {
        return instance;
    }

    /*
     * Create a new current robot
     */
    public void createRobot(RobotDescription newRobot) {
        this.robot = newRobot;
        RobotId id = robot.getRobotId();
        if(robot.getConnectionStatus()==null) {
            robot.setConnectionStatus(RobotDescription.CONNECTION_STATUS_UNCONNECTED);
        }

        StringBuilder sql = new StringBuilder("INSERT INTO Robots(masterUri,robotName,robotType,");
        sql.append("ssid,application,platform) ");
        sql.append("VALUES(?,?,?,?,?,?)");
        SQLiteDatabase db = dbManager.getWritableDatabase();
        SQLiteStatement stmt = db.compileStatement(sql.toString());
        stmt.bindString(1,id.getMasterUri());
        stmt.bindString(2,robot.getRobotName());
        stmt.bindString(3,robot.getRobotType());
        stmt.bindString(4,id.getSSID());
        stmt.bindString(5,robot.getApplicationName());
        stmt.bindString(6,robot.getPlatform());
        stmt.executeInsert();
        stmt.close();
    }

    /**
     * Only one robot description is allowed, so we simply delete then insert.
     * @param updatedRobot the modified robot definition
     */
    public void updateRobot(RobotDescription updatedRobot) {
        dbManager.execSQL("DELETE FROM Robots");
        createRobot(updatedRobot);

    }

    public void clearRobot() {
        String sql = "DELETE FROM Robots";
        dbManager.execSQL(sql);
        this.robot = null;
    }

    public RobotDescription getRobot() { return this.robot; }
    public String getConnectionStatus() {
        if( robot!=null ) return robot.getConnectionStatus();
        return RobotDescription.CONNECTION_STATUS_UNCONNECTED;
    }
    public void setConnectionStatus(String status) {
        if( robot!=null ) robot.setConnectionStatus(status);
    }

    /**
     * Initialize the robot object from the database.
     * @return a robot initialized from the database.
     */
    private RobotDescription initializeRobot() {
        RobotDescription r = null;
        SQLiteDatabase db = dbManager.getReadableDatabase();
        StringBuilder sql = new StringBuilder(
                   "SELECT masterUri,robotName,robotType,ssid,application,platform");
        sql.append(" FROM Robots ");
        sql.append(" ORDER BY robotName");
        Cursor cursor = db.rawQuery(sql.toString(),null);
        cursor.moveToFirst();
        while(!cursor.isAfterLast()) {
            RobotId id = new RobotId(cursor.getString(0));
            id.setSSID(cursor.getString(3));
            r = new RobotDescription(id, cursor.getString(1), cursor.getString(2), new Date());
            r.setConnectionStatus(RobotDescription.CONNECTION_STATUS_UNCONNECTED);
            cursor.moveToNext();
        }
        cursor.close();
        return r;
    }

    // Set the SSID of the current robot and update database.
    public void setSSID(String ssid) {
        if( this.robot!=null ) {
            this.robot.getRobotId().setSSID(ssid);
            updateRobot(this.robot);
        }
    }

}
