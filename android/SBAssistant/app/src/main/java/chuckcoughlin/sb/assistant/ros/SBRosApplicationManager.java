/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * Re-create ROS App objects from the database.
 * (MIT License)
 */
package chuckcoughlin.sb.assistant.ros;

import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.os.Handler;
import android.util.Log;

import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.exception.RosException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


import chuckcoughlin.sb.assistant.db.SBDbManager;
import ros.android.appmanager.SBRobotConnectionErrorListener;
import ros.android.msgs.Topic;
import ros.android.util.RobotApplication;
import ros.android.util.RobotDescription;


/**
 * Since we access from multiple fragments, make this a singleton class to avoid repeated
 * allocations. This class encapsulates the node connected to the robot. Individual panels
 * make subscriptions and publications as appropriate.
 *
 * The list of recognized applications is in the database and is hard-coded.  We keep the current
 * application and its running status locally.
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

    // Listeners for application status
    private final ListenerGroup<SBApplicationStatusListener> applicationListeners;
    private final ListenerGroup<SBRobotConnectionErrorListener> errorListeners;


    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * @param context
     */
    private SBRosApplicationManager(Context context) {
        this.context = context.getApplicationContext();
        this.dbManager = SBDbManager.getInstance();
        this.rosManager = SBRosManager.getInstance();
        this.application = null;
        this.applicationListeners = new ListenerGroup(Executors.newSingleThreadExecutor());
        this.errorListeners = new ListenerGroup(Executors.newSingleThreadExecutor());
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
     * Specify the name of a the current application. From this we create a RobotApplication
     * object encapsulating a connected node. The connected node is a departure point for
     * making supscriptions and publication requests.
     * @param appName name of the application
     */
    public void createApplicationFromName(String appName) {
        this.application = null;
        RobotDescription robot = rosManager.getRobot();
        if (robot != null) {
            SQLiteDatabase db = dbManager.getReadableDatabase();

            String sql = String.format("SELECT description FROM RobotApplications WHERE appName='%s'",appName);
            String[] args = new String[]{};

            Cursor cursor = db.rawQuery(sql, args);
            cursor.moveToFirst();
            String description = "Unconfigured";
            if (!cursor.isAfterLast()) {
                description = cursor.getString(0);

            }
            cursor.close();

            //
            this.application = new RobotApplication(appName,description);
            application.setExecutionStatus(RobotApplication.APP_STATUS_NOT_RUNNING);
        };
    }

    public void startApplication() {
        if( this.application!=null ) {

        }

    }

    public void stopApplication() {
        application.setExecutionStatus(RobotApplication.APP_STATUS_NOT_RUNNING);
        signalApplicationStop();
    }

    /**
     * The robot has been disconnected.
     */
    public void shutdown() {
        stopApplication();
        application = null;
    }

    /**
     * Create a new ROS NodeConfiguration object.
     * @param masterUri string version of the master URI
     * @return the new configuration
     * @throws RosException If masterUri is invalid or if we cannot get a hostname for the
     *                      device we are running on.
     */
    private NodeConfiguration createConfiguration(String masterUri) throws RosException {
        String namespace = "/";
        HashMap<GraphName, GraphName> remappings = new HashMap<GraphName, GraphName>();
        NameResolver resolver = new NameResolver(GraphName.of(namespace), remappings);

        URI uri;
        try {
            uri = new URI(masterUri);
        }
        catch (URISyntaxException e) {
            Log.i(CLSS, "createConfiguration(" + masterUri + ") invalid master uri.");
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




    public void addListener(SBRobotConnectionErrorListener listener) {
        this.errorListeners.add(listener);
    }

    public void addListener(SBApplicationStatusListener listener) {
        this.applicationListeners.add(listener);
    }
    public void removeListeners() {
        this.applicationListeners.shutdown();
    }

    private void signalApplicationStart(String appName) {
        this.applicationListeners.signal(new SignalRunnable<SBApplicationStatusListener>() {
            public void run(SBApplicationStatusListener listener) {
                listener.applicationStarted(appName);
            }
        });
    }
    /**
     * Inform all listeners that the application has stopped. It is up to the
     * individual listeners to terminate all active subscriptions.
     */
    private void signalApplicationStop() {
        this.applicationListeners.signal(new SignalRunnable<SBApplicationStatusListener>() {
            public void run(SBApplicationStatusListener listener) {
                listener.applicationShutdown();
            }
        });
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
