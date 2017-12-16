/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *   This class functions as the ros.android.util.MasterChooser
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.ros;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteStatement;
import android.net.Uri;
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
import java.util.ArrayList;
import java.util.Date;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import chuckcoughlin.sb.assistant.db.SBDbHelper;
import ros.android.util.InvalidRobotDescriptionException;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;
import ros.android.util.RobotsContentProvider;

/**
 * Since we access from multiple fragments, make this a singleton class to avoid repeated
 * allocations. This class makes the list of robots and the current robot available.
 */
public class SBRosHelper  {
    private final static String CLSS = "SBRosHelper";
    private static SBRosHelper instance = null;
    private final SBDbHelper dbHelper;
    private final Context context;
    private RobotDescription currentRobot = null;

    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * @param context
     */
    private SBRosHelper(Context context) {
        this.context = context.getApplicationContext();
        this.dbHelper= SBDbHelper.getInstance();
    }

    /**
     * Use this method in the initial activity. We need to assign the context.
     * @param context
     * @return the Singleton instance
     */
    public static synchronized SBRosHelper initialize(Context context) {
        // Use the application context, which will ensure that you
        // don't accidentally leak an Activity's context.
        if (instance == null) {
            instance = new SBRosHelper(context.getApplicationContext());
        }
        return instance;
    }

    /**
     * Use this method for all subsequent calls. We often don't have
     * a convenient context.
     * @return the Singleton instance.
     */
    public static synchronized SBRosHelper getInstance() {
        return instance;
    }


    public void addRobot(RobotDescription robot) {
        RobotId id = robot.getRobotId();
        StringBuilder sql = new StringBuilder("INSERT INTO Robots(masterUri,robotName,robotType,");
        sql.append("controlUri,wifi,wifiEncryption ,wifiPassword ,connectionStatus) ");
        sql.append("VALUES(?,?,?,?,?,?,?,?)");
        SQLiteDatabase db = dbHelper.getWritableDatabase();
        SQLiteStatement stmt = db.compileStatement(sql.toString());
        stmt.bindString(1,id.getMasterUri());
        stmt.bindString(2,robot.getRobotName());
        stmt.bindString(3,robot.getRobotType());
        stmt.bindString(4,id.getControlUri());
        stmt.bindString(5,id.getWifi());
        stmt.bindString(6,id.getWifiEncryption());
        stmt.bindString(7,id.getWifiPassword());
        stmt.bindString(8,robot.getConnectionStatus());
        stmt.executeInsert();
    }
    public void clearRobots() {
        String sql = "DELETE FROM Robots";
        dbHelper.execSQL(sql);
    }
    public RobotDescription getCurrentRobot() { return this.currentRobot; }
    public List<RobotDescription> getRobots() {
        List<RobotDescription> robots = new ArrayList<>();
        SQLiteDatabase db = dbHelper.getReadableDatabase();
        String table = "Robots";
        String[] columns = { "masterUri","robotName","robotType","controlUri","wifi","wifiEncryption","wifiPassword","connectionStatus" };
        Cursor cursor = db.query(table, columns, null, null, null, null, "robotName");
        cursor.moveToFirst();
        while(!cursor.isAfterLast()) {
            Map<String,Object> map = new HashMap<>();
            map.put("URL",cursor.getString(1));
            map.put("CURL",cursor.getString(4));
            map.put("WIFI",cursor.getString(5));
            map.put("WIFIENC",cursor.getString(6));
            map.put("WIFIPW",cursor.getString(7));
            RobotId id = new RobotId(map);
            try {
                RobotDescription robot = new RobotDescription(id, cursor.getString(2), cursor.getString(3), new Date());
                robots.add(robot);
            }
            catch(InvalidRobotDescriptionException irde) {
                Log.i(CLSS, String.format("getRobots %s caught InvalidRobotDescriptionException: %s ",cursor.getString(2),irde.getMessage()));
            }
            cursor.moveToNext();
        }
        return robots;
    }
    public void setCurrentRobot(RobotDescription description) { this.currentRobot = description; }
    public void setCurrentRobot(int position) {
        List<RobotDescription> robots = getRobots();
        if( position<robots.size() ) {
            this.currentRobot = robots.get(position);
        }
    }

    /***
     * Replace the list of robots with the list specified.
     * @param robots
     */
    public void setRobots(List<RobotDescription> robots) {
        clearRobots(); // Start clean.
        for( RobotDescription robot:robots) {
            addRobot(robot);
        }
    }
    /**
     * Returns true if current master URI and robot name are set in memory, false
     * otherwise. Does not read anything from disk.
     */
    public boolean hasRobot() {
        return (currentRobot != null && currentRobot.getRobotId() != null
                && currentRobot.getRobotId().getMasterUri() != null
                && currentRobot.getRobotId().getMasterUri().toString().length() != 0
                && currentRobot.getRobotName() != null && currentRobot.getRobotName().length() != 0);
    }




    /**
     * Create and return a new ROS NodeContext object based on the current value
     * of the internal masterUri variable.
     *
     * @throws RosException If the master URI is invalid or if we cannot get a hostname for
     *                      the device we are running on.
     */
    public NodeConfiguration createConfiguration() throws RosException {
        return createConfiguration(currentRobot.getRobotId());
    }



    /**
     * Create and return a new ROS NodeContext object.
     *
     * @throws RosException If masterUri is invalid or if we cannot get a hostname for the
     *                      device we are running on.
     */
    public NodeConfiguration createConfiguration(RobotId robotId) throws RosException {
        Log.i(CLSS, "createConfiguration(" + robotId.toString() + ")");
        if (robotId == null || robotId.getMasterUri() == null) {
            // TODO: different exception type for invalid master uri
            throw new RosException("ROS Master URI is not set");
        }
        String namespace = "/";
        HashMap<GraphName, GraphName> remappings = new HashMap<GraphName, GraphName>();
        NameResolver resolver = new NameResolver(GraphName.of(namespace), remappings);

        URI uri;
        try {
            uri = new URI(robotId.getMasterUri());
        } catch (URISyntaxException e) {
            Log.i(CLSS, "createConfiguration(" + robotId.toString() + ") invalid master uri.");
            throw new RosException("Invalid master URI");
        }

        Log.i(CLSS, "createConfiguration() creating configuration.");
        NodeConfiguration configuration =
                NodeConfiguration.newPublic(getNonLoopbackHostName());
        configuration.setParentResolver(resolver);
        configuration.setRosPackagePath(null);
        configuration.setMasterUri(uri);

        Log.i(CLSS, "createConfiguration() returning configuration with host = " + getNonLoopbackHostName());
        return configuration;
    }




    /**
     * @return The first valid non-loopback, IPv4 host name (address in text form
     * like "10.0.129.222" found for this device.
     */
    private static String getNonLoopbackHostName() {
        Log.i(CLSS, "getNonLoopbackHostName() starts");
        try {
            String address = null;
            for (Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces(); en
                    .hasMoreElements(); ) {
                NetworkInterface intf = en.nextElement();
                Log.i(CLSS, "getNonLoopbackHostName() sees interface: " + intf.getName());
                for (Enumeration<InetAddress> enumIpAddr = intf.getInetAddresses(); enumIpAddr
                        .hasMoreElements(); ) {
                    InetAddress inetAddress = enumIpAddr.nextElement();
                    Log.i(CLSS, "getNonLoopbackHostName() sees address: " + inetAddress.getHostAddress().toString());
                    // IPv4 only for now
                    if (!inetAddress.isLoopbackAddress() && inetAddress.getAddress().length == 4) {
                        if (address == null)
                            address = inetAddress.getHostAddress().toString();
                    }
                }
            }
            if (address != null) {
                Log.i(CLSS, "getNonLoopbackHostName() returning " + address);
                return address;
            }
        } catch (SocketException ex) {
            Log.i(CLSS, "getNonLoopbackHostName() caught SocketException: " + ex.getMessage());
        }
        Log.i(CLSS, "getNonLoopbackHostName() returning null.");
        return null;
    }

}
