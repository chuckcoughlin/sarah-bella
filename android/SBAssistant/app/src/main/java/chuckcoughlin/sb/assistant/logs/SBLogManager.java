package chuckcoughlin.sb.assistant.logs;

import org.ros.node.ConnectedNode;

import java.util.ArrayList;
import java.util.List;

import chuckcoughlin.sb.assistant.common.AbstractMessageListener;
import chuckcoughlin.sb.assistant.common.FixedSizeList;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBApplicationStatusListener;
import chuckcoughlin.sb.assistant.ros.SBApplicationManager;
import rosgraph_msgs.Log;

/**
 * The log manager subscribes to log messages whenever any application is active.
 * The instance is created and shutdown in the MainActivity. The instance must be
 * initialized as its first operation.
 */
public class SBLogManager implements SBApplicationStatusListener {
    private final static String CLSS = "SBLogManager";
    private static volatile SBLogManager instance = null;
    private final LogListener logListener;
    private final FixedSizeList<rosgraph_msgs.Log> logList;
    private final List<LogListObserver> observers;

    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * On start, create subscriptions to the applications.
     */
    private SBLogManager() {
        SBApplicationManager.getInstance().addListener(this);
        logList = new FixedSizeList<rosgraph_msgs.Log>(SBConstants.NUM_LOG_MESSAGES);
        logListener = new LogListener();
        observers = new ArrayList<>();
    }

    /**
     * Use this method in the initial activity. We need to assign the context.
     * @return the Singleton instance
     */
    public static synchronized SBLogManager initialize() {
        // Use the application context, which will ensure that you
        // don't accidentally leak an Activity's context.
        if (instance == null) {
            instance = new SBLogManager();
        }
        else {
            throw new IllegalStateException("Attempt to initialize old copy of SBLogManager");
        }
        return instance;
    }

    /**
     * Called when main activity is destroyed. Clean up any resources.
     * To use again requires re-initialization.
     */
    public static void destroy() {
        if (instance != null) {
            synchronized (SBApplicationManager.class) {
                instance.shutdown();
                instance = null;
            }
        }
    }
    /**
     * Use this method for all subsequent calls. We often don't have
     * a convenient context.
     * @return the Singleton instance.
     */
    public static synchronized SBLogManager getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Attempt to return uninitialized copy of SBLogManager");
        }
        return instance;
    }

    public FixedSizeList<rosgraph_msgs.Log> getLogs() { return logList; }
    public Log getLogAtPosition(int pos) {
        Log result = null;
        if(pos>=0 && pos<logList.size()) { result = logList.get(pos); }
        return result;
    }

    private void shutdown() {
        if( SBApplicationManager.getInstance()!=null) {
            SBApplicationManager.getInstance().removeListener(this);
        }
    }

    // =================================== SBApplicationStatusListener ============================
    // We don't care what the application is, subscribe to the logs.
    @Override
    public void applicationStarted(String appName) {
        ConnectedNode node = SBApplicationManager.getInstance().getCurrentApplication().getConnectedNode();
        android.util.Log.i(CLSS, String.format("application started - %s", appName));
        if (node != null) {
            logListener.subscribe(node, "/rosout");  // Aggregated feed
            android.util.Log.i(CLSS, String.format("subscription started - /rosout"));
        }
        else {
            android.util.Log.i(CLSS, String.format("No connected node"));
        }
    }

    @Override
    public void applicationShutdown(String appName) {
        logListener.shutdown();
        android.util.Log.i(CLSS, String.format("application stopped"));
        shutdown();
    }


    public void addObserver(LogListObserver observer) {
        observers.add(observer);
    }
    public void removeObserver(LogListObserver observer) {
        observers.remove(observer);
    }

    /**
     * Notify all observers regarding a new message
     * @param full true if the list was at capacity before getting the current new message
     */
    private void notifyObservers(boolean full) {
        for(LogListObserver observer:observers) {
            if( observer!=null ) {
                if(full) observer.notifyLogRemoved();
                observer.notifyLogAppended();
            }
            else {
                //android.util.Log.i(CLSS, String.format("WARNING: Attempt to notify null observer"));
            }
        }
    }

    // =================================== Message Listener ============================
    private class LogListener extends AbstractMessageListener<rosgraph_msgs.Log> {
        public LogListener() {
            super(rosgraph_msgs.Log._TYPE);
        }

        @Override
        public void onNewMessage(rosgraph_msgs.Log msg) {
            android.util.Log.i(CLSS, String.format("Got a log message - %s", msg.getMsg()));
            boolean full = logList.isFull();
            logList.add(msg);
            notifyObservers(full);
        }
    }
}
