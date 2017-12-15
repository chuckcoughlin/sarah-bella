package chuckcoughlin.sb.assistant.ros;

import android.content.Context;
import android.database.Cursor;
import android.database.SQLException;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.util.Log;

import java.util.ArrayList;
import java.util.List;

import chuckcoughlin.sb.assistant.utilities.NameValue;
import chuckcoughlin.sb.assistant.utilities.SBConstants;

/**
 * Since we access from multiple fragments, make this a singleton class to avoid repeated
 * allocations. This class makes the current robot available.
 */
public class SBRosHelper  {
    private final static String CLSS = "SBRosHelper";
    private static SBRosHelper instance = null;
    private final Context context;

    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * @param context
     */
    private SBRosHelper(Context context) {
        this.context = context.getApplicationContext();
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

}
