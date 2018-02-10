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
import android.os.StrictMode;
import android.util.Log;

import org.ros.RosCore;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.exception.RosException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


import chuckcoughlin.sb.assistant.db.SBDbManager;
import ros.android.appmanager.SBRobotConnectionErrorListener;
import ros.android.msgs.Topic;
import ros.android.util.RobotApplication;
import ros.android.util.RobotDescription;


/**
 * Encapsulate the current application running on the tablet. It must match the application
 * running on the robot to which we are connected.
 * Since we access from multiple fragments, make this a singleton class to avoid repeated
 * allocations. It is created and shutdown in the MainActivity.
 *
 * This class also encapsulates the node connected to the robot. Individual panels
 * make subscriptions and publications to it as appropriate.
 *
 * The list of recognized applications is in the database and is hard-coded.  We keep the current
 * application and its running status locally. Listeners are informed appropriately.
 *
 * We start RosCore when the application is started and stop it when the application stops.
 */
public class SBRosApplicationManager {
    private final static String CLSS = "SBRosManager";

    private static volatile SBRosApplicationManager instance = null;



    private Thread nodeThread;
    private Handler uiThreadHandler = new Handler();
    private RobotApplication application;
    private final List<RobotApplication> apps;
    private RosCore rosCore = null;
    private NodeConfiguration nodeConfiguration= null;
    private NodeMainExecutor nodeMainExecutor  = null;

    // Listeners for application status
    private final ListenerGroup<SBApplicationStatusListener> applicationListeners;
    private final ListenerGroup<SBRobotConnectionErrorListener> errorListeners;


    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     */
    private SBRosApplicationManager() {
        this.application = null;
        this.applicationListeners = new ListenerGroup(Executors.newSingleThreadExecutor());
        this.apps = new ArrayList<>();
        this.errorListeners = new ListenerGroup(Executors.newSingleThreadExecutor());
    }

    /**
     * Use this method in the initial activity. We need to assign the context.
     * @return the Singleton instance
     */
    public static SBRosApplicationManager getInstance() {
        if (instance == null) {
            synchronized(SBRosApplicationManager.class) {instance = new SBRosApplicationManager(); }
        }
        return instance;
    }
    public void addListener(SBRobotConnectionErrorListener listener) {
        this.errorListeners.add(listener);
    }
    /**
     * The new listener may have missed the start of the application, so send it on registration.
     * @param listener
     */
    public void addListener(SBApplicationStatusListener listener) {
        if( this.application!=null && application.getExecutionStatus().equalsIgnoreCase(RobotApplication.APP_STATUS_RUNNING)) {
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    listener.applicationStarted(application.getApplicationName());
                }
            });
            thread.start();
        }
        this.applicationListeners.add(listener);
    }

    /**
     * @return the fixed list of applications. This list is independent of the robot instance.
     */
    public RobotApplication getApplication() { return this.application; }
    /**
     * @return the list of known applications
     */
    public List<RobotApplication> getApplications() {
        if( apps.size()==0 ) createApplications();
        return this.apps;
    }

    /**
     * @return the index of the current application in the list of applications
     */
    public int indexOfCurrentApplication() {
        int result = -1;
        if( application!=null ) {
            int index = 0;
            for(RobotApplication app:apps) {
                if( app.getApplicationName().equalsIgnoreCase(application.getApplicationName())) {
                    result = index;
                    break;
                }
                index++;
            }
        }
        return result;
    }

    public void removeListener(SBApplicationStatusListener listener) {
        this.applicationListeners.remove(listener);
    }

    /**
     * Define a new application
     * @param name
     */
    public void setApplication(String name) {
        // Inform any listeners on the old application that it has stopped.
        if( this.application!=null && !this.application.getApplicationName().equalsIgnoreCase(name)) {
            signalApplicationStop();
        }
        this.application = null;
        for (RobotApplication app : apps) {
            if (app.getApplicationName().equalsIgnoreCase(name)) {
                this.application = app;
                break;
            }
        }
    }
    /**
     * Start RosCore. We need the MasterURI. The application gets the ConnectedNode,
     * inform any subscribing panels.
     */
    public void startApplication(RobotDescription robot) {
        if( this.application!=null ) {
            Thread thread = new Thread(new Runnable(){
                @Override
                public void run() {
                    rosCore = RosCore.newPublic(11311);
                    rosCore.start();
                    Log.i(CLSS, "startApplication: started rosCore");
                    try {
                        rosCore.awaitStart();
                        URI masterURI = URI.create(robot.getRobotId().getMasterUri());
                        String localhost = "127.0.0.1";
                        nodeConfiguration = NodeConfiguration.newPublic(localhost, masterURI);
                        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
                        nodeMainExecutor.execute(application,nodeConfiguration);
                        application.setExecutionStatus(RobotApplication.APP_STATUS_RUNNING);
                        signalApplicationStart(application.getApplicationName());
                    }
                    catch (Exception ex) {
                        signalError(String.format("%s.startApplication: Unable to start core (%s), continuing",CLSS,ex.getLocalizedMessage()));
                    }
                }
            });
            thread.start();
        }
    }

    /**
     * Shutdown the application and all its subscribers.
     * Stop Roscore.
     */
    public void stopApplication() {
        if( application==null ) return;
        application.setExecutionStatus(RobotApplication.APP_STATUS_NOT_RUNNING);
        signalApplicationStop();
        if( nodeMainExecutor!=null ) nodeMainExecutor.shutdown();
        rosCore.shutdown();
    }

    private void shutdown() {
        applicationListeners.shutdown();
        errorListeners.shutdown();
    }

    /**
     * Called when main activity is destroyed. Clean up any resources.
     * To use again requires re-initialization.
     */
    public static void destroy() {
        if( instance!=null ) {
            synchronized (SBRosApplicationManager.class) {
                instance.shutdown();
                instance = null;
            }
        }
    }


    /**
     * Create the initial fixed list of applications
     * @return application list.
     */
    private void createApplications() {
        apps.clear();

        SBDbManager dbManager = SBDbManager.getInstance();
        SQLiteDatabase db = dbManager.getReadableDatabase();

        StringBuilder sql = new StringBuilder(
                "SELECT appName,description");
        sql.append(" FROM RobotApplications");
        sql.append(" ORDER BY appName");
        String[] args = new String[]{};
        //Log.i(CLSS,String.format("APPLICATIONS: -------------------------"));
        Cursor cursor = db.rawQuery(sql.toString(), args);
        cursor.moveToFirst();
        while (!cursor.isAfterLast()) {
            //Log.i(CLSS,String.format("Add application %s:%s",cursor.getString(0),cursor.getString(1)));

            RobotApplication app = new RobotApplication(cursor.getString(0),cursor.getString(1));
            app.setExecutionStatus(RobotApplication.APP_STATUS_NOT_RUNNING);
            apps.add(app);
            cursor.moveToNext();
        }
        cursor.close();
    }
    /**
     * Inform all listeners that the named application has started. It is up to the
     * individual listeners to create subscriptions as appropriate.
     */
    private void signalApplicationStart(String appName) {
        Log.i(CLSS, String.format("signalApplicationStart: %s",appName));
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
     * Inform all listeners of an error.
     */
    private void signalError(String msg) {
        this.errorListeners.signal(new SignalRunnable<SBRobotConnectionErrorListener>() {
            public void run(SBRobotConnectionErrorListener listener) {
                listener.handleConnectionError(msg);
            }
        });
    }

    // ======================================== UNUSED ===========================================================

    /**
     * Create a new ROS NodeConfiguration object.  NOT USED
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
