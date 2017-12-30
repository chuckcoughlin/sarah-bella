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
import ros.android.util.InvalidRobotDescriptionException;
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

    // Keys for columns in result map. Somewhere there's a 6 char limit.
    public static final String URI_VALUE        = "URI";
    public static final String CURL_VALUE       ="CURL";
    public static final String WIFI_VALUE       = "WIFI";
    public static final String ENCRYPTION_VALUE = "WIFIENC";
    public static final String PASSWORD_VALUE   = "WIFIPW";
    public static final String GATEWAY_VALUE    = "GATEWY";
    public static final String PLATFORM_VALUE   = "PLTFRM";
    public static final String ICON_PATH_VALUE  = "ICON";

    private static SBRosManager instance = null;
    private final SBDbManager dbManager;
    private final Context context;
    private Thread nodeThread;
    private Handler uiThreadHandler = new Handler();
    private RobotDescription robot = null;


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
     * Valid only if there are no existing robots
     */
    public void createRobot(RobotDescription robot) {
        RobotId id = robot.getRobotId();
        if(robot.getConnectionStatus()==null) {
            robot.setConnectionStatus("");
        }
        StringBuilder sql = new StringBuilder("INSERT INTO Robots(masterUri,robotName,robotType,");
        sql.append("controlUri,wifi,wifiEncryption,wifiPassword,platform,gateway,connectionStatus) ");
        sql.append("VALUES(?,?,?,?,?,?,?,?,?,?)");
        SQLiteDatabase db = dbManager.getWritableDatabase();
        SQLiteStatement stmt = db.compileStatement(sql.toString());
        stmt.bindString(1,id.getMasterUri());
        stmt.bindString(2,robot.getRobotName());
        stmt.bindString(3,robot.getRobotType());
        stmt.bindString(4,id.getControlUri());
        stmt.bindString(5,id.getWifi());
        stmt.bindString(6,id.getWifiEncryption());
        stmt.bindString(7,id.getWifiPassword());
        stmt.bindString(8,robot.getPlatformType());
        stmt.bindString(9,robot.getGatewayName());
        stmt.bindString(10,robot.getConnectionStatus());
        stmt.executeInsert();
        stmt.close();
    }

    /**
     * Only one robot description is allowed, so we simply delete then insert.
     * @param robot the new robot definition
     */
    public void updateRobot(RobotDescription robot) {
        dbManager.execSQL("DELETE FROM Robots");
        createRobot(robot);
    }

    public void clearRobot() {
        String sql = "DELETE FROM Robots";
        dbManager.execSQL(sql);
    }

    public RobotDescription getRobot() { return this.robot; }

    /**
     * Initialize the robot object from the database.
     * @return a robot initialized from the database.
     */
    private RobotDescription initializeRobot() {
        RobotDescription r = null;
        SQLiteDatabase db = dbManager.getReadableDatabase();
        StringBuilder sql = new StringBuilder(
                   "SELECT masterUri,robotName,robotType,controlUri,wifi,wifiEncryption,wifiPassword,platform,gateway,connectionStatus");
        sql.append(" FROM Robots ");
        sql.append(" ORDER BY robotName");
        Cursor cursor = db.rawQuery(sql.toString(),null);
        cursor.moveToFirst();
        while(!cursor.isAfterLast()) {
            Map<String,Object> map = new HashMap<>();
            map.put(URI_VALUE,cursor.getString(0));
            map.put(CURL_VALUE,cursor.getString(3));
            map.put(WIFI_VALUE,cursor.getString(4));
            map.put(ENCRYPTION_VALUE,cursor.getString(5));
            map.put(PASSWORD_VALUE,cursor.getString(6));
            map.put(PLATFORM_VALUE,cursor.getString(7));
            map.put(GATEWAY_VALUE,cursor.getString(8));
            RobotId id = new RobotId(map);
            try {
                r = new RobotDescription(id, cursor.getString(1), cursor.getString(2), new Date());
            }
            catch(InvalidRobotDescriptionException irde) {
                Log.i(CLSS, String.format("getRobots %s caught InvalidRobotDescriptionException: %s ",cursor.getString(2),irde.getMessage()));
            }
            cursor.moveToNext();
        }
        cursor.close();
        return r;
    }




    /**
     * Create and return a new ROS NodeContext object based on the current value
     * of the internal masterUri variable.
     *
     * @throws RosException If the master URI is invalid or if we cannot get a hostname for
     *                      the device we are running on.
     */
    public NodeConfiguration createConfiguration() throws RosException {
        return createConfiguration(robot.getRobotId());
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
