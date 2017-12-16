/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *   This class functions as the ros.android.util.MasterChooser
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.ros;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.net.Uri;
import android.util.Log;

import org.ros.exception.RosException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.yaml.snakeyaml.Yaml;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

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
    private final Context context;
    private RobotDescription currentRobot = null;
    private List<RobotDescription>robots;

    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * @param context
     */
    private SBRosHelper(Context context) {
        this.context = context.getApplicationContext();
        this.robots = new ArrayList<RobotDescription>();
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

    public void addRobot(RobotDescription robot) { this.robots.add(robot); }
    public RobotDescription getCurrentRobot() { return this.currentRobot; }
    public List<RobotDescription> getRobots() { return this.robots; }
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



    @SuppressWarnings("unchecked")
    public List<RobotDescription> readRobotList() {
        String str = null;
        List<RobotDescription>robotList = new ArrayList<RobotDescription>();
        Cursor c = context.getContentResolver().query(RobotsContentProvider.CONTENT_URI, null, null, null, null);
        if (c == null) {
            Log.e(CLSS, "No robots returned by content provider");
        }
        else if (c.getCount() > 0) {
            c.moveToFirst();
            str = c.getString(c.getColumnIndex(RobotsContentProvider.TABLE_COLUMN));
            Log.i(CLSS, "Found  robot: " + str);
            if (str != null) {
                Yaml yaml = new Yaml();
                robotList = (List<RobotDescription>) yaml.load(str);
            }
        }
        return robotList;
    }
    public void setCurrentRobot(int position) {
        if( position<robots.size() ) {
            this.currentRobot = robots.get(position);
        }
    }

    public void writeRobotList(List<RobotDescription> robotList) {
        Log.i(CLSS, "Saving robots ...");
        Yaml yaml = new Yaml();
        String txt = null;
        final List<RobotDescription> finalRobotList = robotList;  //Avoid race conditions
        if (finalRobotList != null) {
            txt = yaml.dump(finalRobotList);
        }
        ContentValues cv = new ContentValues();
        cv.put(RobotsContentProvider.TABLE_COLUMN, txt);
        Uri newEmp = context.getContentResolver().insert(RobotsContentProvider.CONTENT_URI, cv);
        if (newEmp != RobotsContentProvider.CONTENT_URI) {
            Log.e(CLSS, "Could not save robots, non-equal URI's");
        }
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
    static public NodeConfiguration createConfiguration(RobotId robotId) throws RosException {
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



    public void deleteAllRobots() {
        robots.clear();
        //fireRobotsChanged();
        this.currentRobot = null;
        //saveCurrentRobot();
    }
    public void deleteSelectedRobots(boolean[] array) {
        int j=0;
        for (int i=0; i<array.length; i++) {
            if (array[i]) {
                if( robots.get(j).equals( getCurrentRobot() )) {
                    this.currentRobot = null;
                    //saveCurrentRobot();
                }
                robots.remove(j);
            }
            else {
                j++;
            }
        }
    }
    public void deleteUnresponsiveRobots() {
        Iterator<RobotDescription> iter = robots.iterator();
        while (iter.hasNext()) {
            RobotDescription robot = iter.next();
            if (robot == null || robot.getConnectionStatus() == null
                    || robot.getConnectionStatus().equals(robot.ERROR)) {
                Log.i("RosAndroid", "Removing robot with connection status '" + robot.getConnectionStatus()
                        + "'");
                iter.remove();
                if( robot != null && robot.equals( getCurrentRobot() )) {
                    this.currentRobot = null;
                    //saveCurrentRobot();
                }
            }
        }
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
