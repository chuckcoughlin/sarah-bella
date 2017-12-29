package chuckcoughlin.sb.assistant.db;

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
 * Since we access from multiple fragments, make this a singleton class to avoid repeated
 * allocations. We encapsulate all the database operations here.
 */
public class SBDbManager extends SQLiteOpenHelper {
    private final static String CLSS = "SBDbManager";
    private static SBDbManager instance = null;
    private final Context context;

    /**
     * Constructor is private per Singleton pattern. This forces use of the single instance.
     * @param context main activity
     */
    private SBDbManager(Context context) {
        super(context, SBConstants.DB_NAME, null, SBConstants.DB_VERSION);
        this.context = context.getApplicationContext();
    }

    /**
     * Use this method in the initial activity. We need to assign the context.
     * @param context main activity
     * @return the Singleton instance
     */
    public static synchronized SBDbManager initialize(Context context) {
        // Use the application context, which will ensure that you
        // don't accidentally leak an Activity's context.
        if (instance == null) {
            instance = new SBDbManager(context.getApplicationContext());
        }
        return instance;
    }

    /**
     * Use this method for all subsequent calls. We often don't have
     * a convenient context.
     * @return the Singleton instance.
     */
    public static synchronized SBDbManager getInstance() {
        return instance;
    }

    /**
     * Called when the database connection is being configured.
     * Configure database settings for things like foreign key support, write-ahead logging, etc.
     */
    @Override
    public void onConfigure(SQLiteDatabase db) {
        super.onConfigure(db);
        db.setForeignKeyConstraintsEnabled(true);
    }

    /**
     * Called when the database is created for the FIRST time.
     * If a database already exists on disk with the same DATABASE_NAME, this method will NOT be called.
     * @param sqLiteDatabase
     */
    @Override
    public void onCreate(SQLiteDatabase sqLiteDatabase) {
        StringBuilder SQL = new StringBuilder();
        SQL.append("CREATE TABLE IF NOT EXISTS Settings (");
        SQL.append("  name TEXT PRIMARY KEY,");
        SQL.append("  value TEXT DEFAULT ''");
        SQL.append(")");
        sqLiteDatabase.execSQL(SQL.toString());

        SQL = new StringBuilder();
        SQL.append("CREATE TABLE IF NOT EXISTS Robots (");
        SQL.append("  masterUri TEXT PRIMARY KEY,");
        SQL.append("  robotName TEXT DEFAULT '',");
        SQL.append("  robotType TEXT DEFAULT '',");
        SQL.append("  controlUri TEXT DEFAULT '',");
        SQL.append("  wifi TEXT DEFAULT '',");
        SQL.append("  wifiEncryption TEXT DEFAULT '',");
        SQL.append("  wifiPassword TEXT DEFAULT '',");
        SQL.append("  gateway TEXT DEFAULT '',");
        SQL.append("  platform TEXT DEFAULT '',");
        SQL.append("  connectionStatus TEXT DEFAULT ''");
        SQL.append(")");
        sqLiteDatabase.execSQL(SQL.toString());

        // masterUri and appName constitute primary key
        SQL = new StringBuilder();
        SQL.append("CREATE TABLE IF NOT EXISTS RobotApplications (");
        SQL.append("  masterUri TEXT NOT NULL,");
        SQL.append("  appName TEXT NOT NULL,");
        SQL.append("  displayName TEXT NOT NULL,");
        SQL.append("  PRIMARY KEY (masterUri,appName)");
        SQL.append(")");
        sqLiteDatabase.execSQL(SQL.toString());

        // Add initial rows - fail silently if they exist
        String query = "INSERT INTO Settings(Name,Value) VALUES('"+SBConstants.ROS_HOSTNAME+"','"+SBConstants.DEFAULT_ROS_HOSTNAME+"')";
        execLenient(sqLiteDatabase,query);
        query = "INSERT INTO Settings(Name,Value) VALUES('"+SBConstants.ROS_MASTER_URI+"','"+SBConstants.DEFAULT_ROS_MASTER_URI+"')";
        execLenient(sqLiteDatabase,query);
        Log.i(CLSS,String.format("onCreate: Created %s at %s",SBConstants.DB_NAME,context.getDatabasePath(SBConstants.DB_NAME)));
    }

    /**
     * Trap and log any errors.
     * @param sql
     */
    public void execSQL(String sql) {
        SQLiteDatabase database = this.getWritableDatabase();
        try {
            database.execSQL(sql);
        }
        catch(SQLException sqle) {
            Log.e(CLSS,String.format("execSQL:%s; SQLException ignored (%s)",sql,sqle.getLocalizedMessage()));
        }
    }

    /**
     * Trap and ignore any errors.
     * @param sqLiteDatabase
     * @param sql
     */
    public void execLenient(SQLiteDatabase sqLiteDatabase,String sql) {
        try {
            sqLiteDatabase.execSQL(sql);
        }
        catch(SQLException sqle) {
            Log.i(CLSS,String.format("SQLException ignored (%s)",sqle.getLocalizedMessage()));
        }
    }

    /**
     * Alter an existing database to account for changes as time goes on. This is called if the
     * database is accessed with a newer version than exists on disk.
     * @param sqLiteDatabase the database
     * @param oldVersion version number of the existing installation
     * @param newVersion current version number
     */
    @Override
    public void onUpgrade(SQLiteDatabase sqLiteDatabase, int oldVersion, int newVersion) {
        if( oldVersion==SBConstants.DB_VERSION) return; // Already at latest version
        try {
            onCreate(sqLiteDatabase);
        }
        catch(SQLException sqle) {
            Log.e(CLSS, String.format("onUpgrade: SQLError: %s", sqle.getLocalizedMessage()));
        }
    }
    // ================================================= Robot ===============================
    /**
     * For access to the robot table, see SBRosManager.
     */
    // ================================================ Settings =============================
    /**
     * Read name/value pairs from the database.
     */
    public List<NameValue> getSettings() {
        List<NameValue> list = new ArrayList<>();
        SQLiteDatabase database = this.getReadableDatabase();
        String[] args = new String[0];   // Use for PreparedStatement
        String SQL = "SELECT name,value FROM Settings ORDER BY Name";
        Cursor cursor = database.rawQuery(SQL,args);
        cursor.moveToFirst();

        while( !cursor.isAfterLast() ) {
            NameValue nv = new NameValue();
            nv.setName(cursor.getString(0));
            nv.setValue(cursor.getString(1));
            Log.i(CLSS,String.format("getSettings: %s = %s",nv.getName(),nv.getValue()));
            list.add(nv);
            cursor.moveToNext();
        }
        cursor.close();
        database.close();
        return list;
    }
    /**
     * Save a single setting to the database.
     * @param nv the subject name-value pair
     */
    public void updateSetting(NameValue nv) {
        SQLiteDatabase database = this.getWritableDatabase();
        String SQL = "UPDATE Settings set value = ? WHERE name = ?";
        String[] bindArgs = new String[2];

        bindArgs[0] = nv.getValue();
        bindArgs[1] = nv.getName();
        database.execSQL(SQL,bindArgs);
        database.close();
    }

    /**
     * Save settings to the database
     * @param items a list of name-value pairs
     */
    public void updateSettings(List<NameValue> items) {
        SQLiteDatabase database = this.getWritableDatabase();
        String SQL = "UPDATE Settings set value = ? WHERE name = ?";
        String[] bindArgs = new String[2];
        int count = items.size();
        int index = 0;
        while( index<count) {
            NameValue nv = items.get(index);
            bindArgs[0] = nv.getValue();
            bindArgs[1] = nv.getName();
            database.execSQL(SQL,bindArgs);
            index++;
        }
        database.close();
    }
}
