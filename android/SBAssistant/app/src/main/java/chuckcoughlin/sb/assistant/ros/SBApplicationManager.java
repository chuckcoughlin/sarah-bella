/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * Re-create ROS App objects from the database.
 * (MIT License)
 */
package chuckcoughlin.sb.assistant.ros;

import android.database.Cursor;
import android.database.SQLException;
import android.database.sqlite.SQLiteDatabase;
import android.os.AsyncTask;
import android.os.Handler;
import android.util.Log;

import org.ros.RosCore;
import org.ros.address.InetAddressFactory;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.exception.RosException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
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
import java.util.concurrent.Executors;


import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.db.SBDbManager;
import ros.android.appmanager.SBRobotConnectionErrorListener;
import ros.android.util.TabletApplication;
import ros.android.util.RobotDescription;


/**
 * Encapsulate the current application running on the tablet. It must match the application
 * running on the robot to which we are connected.
 * Since we access this from multiple fragments, make this a singleton class to avoid repeated
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
public class SBApplicationManager {
    private final static String CLSS = "SBApplicationManager";
    private static volatile SBApplicationManager instance = null;
    private String localhost = "127.0.0.1";   // Unused
    private Thread nodeThread;
    private Handler uiThreadHandler = new Handler();
    private final List<TabletApplication> apps; // Map by name
    private TabletApplication currentApplication = null;
    private NodeConfiguration nodeConfiguration = null;
    private NodeMainExecutor nodeMainExecutor = null;
    private RosCore rosCore = null;

    // Listeners for application status
    private final ListenerGroup<SBApplicationStatusListener> applicationListeners;
    private final ListenerGroup<SBRobotConnectionErrorListener> errorListeners;

    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     *      Get the URI from the database.
     */
    private SBApplicationManager() {
        this.applicationListeners = new ListenerGroup(Executors.newSingleThreadExecutor());
        this.errorListeners = new ListenerGroup(Executors.newSingleThreadExecutor());
        // Run an async task to deth the node configuration
        new NodeConfigurationTask().execute();
        this.apps     = new ArrayList<>();
    }

    /**
     * Use this method in the initial activity. We need to assign the context.
     * @return the Singleton instance
     */
    public static SBApplicationManager getInstance() {
        if (instance == null) {
            synchronized(SBApplicationManager.class) {instance = new SBApplicationManager(); }
        }
        return instance;
    }
    public void addListener(SBRobotConnectionErrorListener listener) {
        this.errorListeners.add(listener);
    }
    /**
     * The new listener may have missed the starting of the application, so send it on registration.
     * @param listener
     */
    public void addListener(SBApplicationStatusListener listener) {
        this.applicationListeners.add(listener);
        if( this.currentApplication!=null && currentApplication.getExecutionStatus().equalsIgnoreCase(TabletApplication.STATE_ACTIVE)) {
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    listener.applicationStarted(currentApplication.getApplicationName());
                }
            });
            thread.start();
        }
    }

    /**
     * @return the application.
     */
    public TabletApplication getCurrentApplication() { return this.currentApplication; }

    /**
     * @return the fixed (in database) list of known applications.
     *          This list is independent of the robot instance.
     *          Names are ordered alphabetically.
     */
    public List<TabletApplication> getApplications() {
        if( apps.size()==0 ) createApplications();
        return this.apps;
    }

    /**
     * @return the index of the current application in the list of applications
     */
    public int indexOfCurrentApplication() {
        int result = -1;
        if( currentApplication!=null ) {
            int index = 0;
            for(TabletApplication app:apps) {
                if( app.getApplicationName().equalsIgnoreCase(currentApplication.getApplicationName())) {
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
     * Select a different current application.
     * @param name
     */
    public void setApplication(String name) {
        if( currentApplication != null ) {
            if( name!=null && this.currentApplication.getApplicationName().equalsIgnoreCase(name)) return; // no-op
            // Inform any listeners on the old application that it has stopped.
            currentApplication.onShutdown(currentApplication.getConnectedNode());
            signalApplicationStop(currentApplication.getApplicationName());
        }
        this.currentApplication = null;
        for( TabletApplication app : apps ) {
            if( app.getApplicationName().equalsIgnoreCase(name) ) {
                this.currentApplication = app;
                break;
            }
        }
    }

    /**
     * Set as a result of the asynch task on startup.
     * @param config
     */
    public void setNodeConfiguration(NodeConfiguration config) { this.nodeConfiguration=config; }

    /**
     * Start RosCore on the local machine for the current application. The current state
     * must be IDLE. The application encapsulates the ConnectedNode.
     * When the connected node starts, the TabletApplication's onStart method is called.
     */
    public void startApplication() {
        if( this.currentApplication!=null ) {
            if( nodeConfiguration !=null ) {
                if (currentApplication.getExecutionStatus().equalsIgnoreCase(TabletApplication.STATE_IDLE)) {
                    currentApplication.onInitialize(); // State now INITIALIZING

                    currentApplication.onInitialize(); // State now INITIALIZING
                    Thread thread = new Thread(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                rosCore = RosCore.newPublic(11311);
                                rosCore.start();
                                Log.i(CLSS, "startApplication: started rosCore");
                                rosCore.awaitStart();
                                nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
                                nodeMainExecutor.execute(currentApplication, nodeConfiguration);
                                SBApplicationManager.getInstance().signalApplicationStart(currentApplication.getApplicationName());
                            } catch (Exception ex) {
                                signalError(String.format("%s.startApplication: Unable to start (restart?) core (%s), continuing", CLSS, ex.getLocalizedMessage()));
                            }
                        }
                    });
                    thread.start();
                }
                else {
                    Log.w(CLSS, String.format("%s.startApplication: Application %s not started. Current state = %s.", CLSS,currentApplication.getApplicationName(),
                            currentApplication.getExecutionStatus()));
                }
            }
            else {
                Log.w(CLSS,String.format("%s.startApplication: No node configuration has been defined."));
            }
        }
        else {
            Log.w(CLSS,String.format("%s.startApplication: No current application is defined."));
        }
    }

    /**
     * Shutdown the current application and all its subscribers.
     * Stop Roscore.
     */
    public void stopApplication() {
        if( currentApplication==null ) return;
        signalApplicationStop(currentApplication.getApplicationName());
        if( nodeMainExecutor!=null ) nodeMainExecutor.shutdown();
        rosCore.shutdown();
        currentApplication.onShutdown(currentApplication.getConnectedNode());
        rosCore = null;
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
            synchronized (SBApplicationManager.class) {
                instance.shutdown();
                instance = null;
            }
        }
    }


    /**
     * Create the initial fixed map of applications. As we go through the database also
     * create an ordered list of application names
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
        Cursor cursor = null;
        try {
            cursor = db.rawQuery(sql.toString(), args);
            cursor.moveToFirst();
            while (!cursor.isAfterLast()) {
                //Log.i(CLSS,String.format("Add application %s:%s",cursor.getString(0),cursor.getString(1)));

                TabletApplication app = new TabletApplication(cursor.getString(0), cursor.getString(1));
                apps.add(app);
                cursor.moveToNext();
            }
        }
        catch(SQLException sqle ) {
            Log.w(CLSS,String.format("%s.createApplications: ERROR retrieving app names (%s).",sqle.getLocalizedMessage()));
        }
        finally {
            if( cursor!=null ) cursor.close();
        }

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
    private void signalApplicationStop(String appName) {
        this.applicationListeners.signal(new SignalRunnable<SBApplicationStatusListener>() {
            public void run(SBApplicationStatusListener listener) {
                listener.applicationShutdown(appName);
            }
        });
    }
    /**
     * Inform all listeners of an error.
     */
    private void signalError(String msg) {
        this.errorListeners.signal(new SignalRunnable<SBRobotConnectionErrorListener>() {
            public void run(SBRobotConnectionErrorListener listener) {
                listener.handleRobotCommunicationError(msg);
            }
        });
    }

    // ============================= Async Task =============================
    // Get the network connection in an asynchronous task to maintain response in main thread.
    class NodeConfigurationTask extends AsyncTask<Void, Void, NodeConfiguration> {

        private Exception exception;

        protected NodeConfiguration doInBackground(Void ... voids) {
            try {
                String uriString = SBDbManager.getInstance().getSetting(SBConstants.ROS_MASTER_URI);
                URI masterURI = URI.create(uriString);
                String host = InetAddressFactory.newNonLoopback().getHostName();
                NodeConfiguration config = NodeConfiguration.newPublic(host, masterURI);
                return config;
            }
            catch (Exception ex) {
                this.exception = ex;
                Log.w(CLSS,String.format("%s.createApplications: ERROR determining node configuration (%s).",ex.getLocalizedMessage()));
                return null;
            }
            finally{}
        }

        protected void onPostExecute(NodeConfiguration config) {
            SBApplicationManager.getInstance().setNodeConfiguration(config);
        }
    }
}
