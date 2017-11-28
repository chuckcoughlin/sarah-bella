package chuckcoughlin.sb.assistant.db;

import android.content.Context;
import android.database.SQLException;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.util.Log;

import chuckcoughlin.sb.assistant.utilities.SBConstants;

/**
 * Created by chuckc on 11/26/17.
 */
public class SBDbHelper extends SQLiteOpenHelper {
    private final static String CLSS = "SBDbHelper";

    public SBDbHelper(Context context) {
        super(context, SBConstants.DB_NAME, null, SBConstants.DB_VERSION);
    }

    @Override
    public void onCreate(SQLiteDatabase sqLiteDatabase) {
        StringBuilder SQL = new StringBuilder();
        SQL.append("CREATE TABLE IF NOT EXISTS Settings (");
        SQL.append("  name TEXT DEFAULT '',");
        SQL.append("  value TEXT DEFAULT ''");
        SQL.append(")");
        sqLiteDatabase.execSQL(SQL.toString());
        // Add initial rows
        String query = "INSERT INTO Settings(Name) VALUES('"+SBConstants.ROS_HOSTNAME+"')";
        sqLiteDatabase.execSQL(query);
        query = "INSERT INTO Settings(Name) VALUES('"+SBConstants.ROS_MASTER_URI+"')";
        sqLiteDatabase.execSQL(query);

    }

    /**
     * Alter an existing database to account for changes as time goes on.
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
}
