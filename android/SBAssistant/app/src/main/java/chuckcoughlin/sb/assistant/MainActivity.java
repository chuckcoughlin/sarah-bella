/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *    Code derived from RosActivity.java by Willow Garage, Inc.
 *            (MIT License)
 */

package chuckcoughlin.sb.assistant;

import android.app.Fragment;
import android.app.FragmentTransaction;
import android.os.Bundle;
import android.support.design.widget.FloatingActionButton;
import android.support.v4.view.ViewPager;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.Toast;

import chuckcoughlin.sb.assistant.db.SBDbHelper;
import chuckcoughlin.sb.assistant.ros.SBRosHelper;
import chuckcoughlin.sb.assistant.utilities.SBAddRobotDialog;
import chuckcoughlin.sb.assistant.utilities.SBAlertDialog;
import chuckcoughlin.sb.assistant.utilities.SBProgressDialog;
import chuckcoughlin.sb.assistant.utilities.SBRemoveRobotDialog;


public class MainActivity extends AppCompatActivity {
    private static final String CLSS = "MainActivity";
    private static final String DIALOG_TAG = "dialog";
    /**
     * A specialized {@link android.support.v4.view.PagerAdapter} that will provide
     * fragments for each of the application pages. We use a
     * {@link android.support.v4.app.FragmentStatePagerAdapter} so as to conserve
     * memory if the list of pages is great.
     */
    private MainActivityPagerAdapter pagerAdapter;
    /**
     * The {@link ViewPager} that will host the section contents.
     */
    private ViewPager viewPager;
    private Thread nodeThread;
    private SBRosHelper rosHelper;

    private SBAlertDialog wifiDialog;  // See if the user wants to switch WiFi networks
    private SBAlertDialog evictDialog; // See if the user wants to evict another user.
    private SBAlertDialog errorDialog; // Misc issues.
    private SBProgressDialog progress;
    private SBAddRobotDialog addRobotDialog;
    private SBRemoveRobotDialog removeRobotDialog;

    public MainActivity() {
        Log.i(CLSS,"Constructor ...");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        SBRosHelper.initialize(this.getApplicationContext());
        this.rosHelper = SBRosHelper.getInstance();

        Log.i(CLSS,"onCreate: entering ...");
        setContentView(R.layout.activity_main);
        // Close the soft keyboard - it will still open on an EditText
        this.getWindow().setSoftInputMode(WindowManager.LayoutParams.SOFT_INPUT_STATE_ALWAYS_HIDDEN);

        // Get the ViewPager and set it's PagerAdapter so that it can display items
        ViewPager viewPager = findViewById(R.id.viewpager);
        pagerAdapter = new MainActivityPagerAdapter(getSupportFragmentManager(),getApplicationContext());
        viewPager.setAdapter(pagerAdapter);

        FloatingActionButton fab = (FloatingActionButton) findViewById(R.id.fab);
        fab.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                final Toast toast = Toast.makeText(getBaseContext(),"Hi MOM",Toast.LENGTH_LONG);
                toast.show();
            }
        });

        // If I absolutely have to start over again with the database
        // this.deleteDatabase(SBConstants.DB_NAME);

        // Initialize the Settings database for use elsewhere in the application.
        SBDbHelper.initialize(this);
    }
    @Override
    protected void onPause() {
        super.onPause();
        if (nodeThread != null) {
            nodeThread.interrupt();
            nodeThread = null;
        }
        closeAllDialogs();
    }
    /**
     * Read the current ROS master URI from external storage and set up the ROS
     * node from the resulting node context. If the current master is not set or
     * is invalid, launch the MasterChooserActivity to choose one or scan a new
     * one.
     */
    @Override
    protected void onResume() {
        super.onResume();

        closeAllDialogs();
        createAllDialogs();
    }

    private void closeAllDialogs() {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (wifiDialog != null) {
                    wifiDialog.dismiss();
                }
                if (evictDialog != null) {
                    evictDialog.dismiss();
                }
                if (errorDialog != null) {
                    errorDialog.dismiss();
                }
                if (progress != null) {
                    progress.dismiss();
                }
                if (addRobotDialog != null) {
                    addRobotDialog.dismiss();
                }
                if (removeRobotDialog != null) {
                    removeRobotDialog.dismiss();
                }
            }});
    }

    private void createAllDialogs() {
        wifiDialog = new SBAlertDialog(this,
                new AlertDialog.Builder(this).setTitle("Change Wifi?").setCancelable(false), "Yes", "No");
        evictDialog = new SBAlertDialog(this,
                new AlertDialog.Builder(this).setTitle("Evict User?").setCancelable(false), "Yes", "No");
        errorDialog = new SBAlertDialog(this,
                new AlertDialog.Builder(this).setTitle("Could Not Connect").setCancelable(false), "Ok");
        progress = new SBProgressDialog(this);
        addRobotDialog = new SBAddRobotDialog();
        addRobotDialog.setTitle("Add Robot");
        removeRobotDialog = new SBRemoveRobotDialog(this);
    }

    // ============================================== Menu Handling ===============================================
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.robot_chooser_options_menu, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        if (id == R.id.add_robot) {
            // DialogFragment.show() will take care of adding the fragment
            // in a transaction.  We also want to remove any currently showing
            // dialog, so make our own transaction and take care of that here.
            FragmentTransaction ft = getFragmentManager().beginTransaction();
            Fragment prev = getFragmentManager().findFragmentByTag("dialog");
            if (prev != null) {
                ft.remove(prev);
            }
            ft.addToBackStack(null);
            addRobotDialog.show(getSupportFragmentManager(),DIALOG_TAG);
            return true;
        }
        else if (id == R.id.delete_selected) {
            removeRobotDialog.show();
            return true;
        }
        else if (id == R.id.delete_unresponsive) {
            SBRosHelper.getInstance().deleteUnresponsiveRobots();
            return true;
        }
        else if (id == R.id.delete_all) {
            SBRosHelper.getInstance().deleteAllRobots();
            return true;
        }
        else if (id == R.id.kill) {
            android.os.Process.killProcess(android.os.Process.myPid());
            return true;
        }
        else {
            return super.onOptionsItemSelected(item);
        }
    }







}
