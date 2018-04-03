package chuckcoughlin.sb.assistant.logs;

import android.content.Context;
import android.database.Cursor;
import android.database.SQLException;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.util.Log;

import java.util.ArrayList;
import java.util.List;

import chuckcoughlin.sb.assistant.common.NameValue;
import chuckcoughlin.sb.assistant.common.SBConstants;

/**
 * The log manager subscribes to log messages whenever any application is active.
 * The instance is created and shutdown in the MainActivity. The instance must be initialized as its first operation.
 */
public class SBLogManager {
    private final static String CLSS = "SBLogManager";
    private static volatile SBLogManager instance = null;
    private volatile Context context = null;

    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * @param context main activity
     */
    private SBLogManager(Context context) {
        this.context = context.getApplicationContext();
    }

    /**
     * Use this method in the initial activity. We need to assign the context.
     * @param context main activity
     * @return the Singleton instance
     */
    public static synchronized SBLogManager initialize(Context context) {
        // Use the application context, which will ensure that you
        // don't accidentally leak an Activity's context.
        if (instance == null) {
            instance = new SBLogManager(context.getApplicationContext());
        }
        else {
            throw new IllegalStateException("Attempt to initialize old copy of SDBManager");
        }
        return instance;
    }

    /**
     * Use this method for all subsequent calls. We often don't have
     * a convenient context.
     * @return the Singleton instance.
     */
    public static synchronized SBLogManager getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Attempt to return uninitialized copy of SDBManager");
        }
        return instance;
    }






    /**
     * Called when main activity is destroyed. Clean up any resources.
     * To use again requires re-initialization.
     */
    public static void destroy() {
        instance = null;
    }
}
