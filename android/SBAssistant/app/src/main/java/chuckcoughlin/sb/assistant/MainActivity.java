/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant;

import android.content.Context;
import android.database.sqlite.SQLiteDatabase;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;

import android.support.v4.app.FragmentPagerAdapter;
import android.support.v4.view.ViewPager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.Toast;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.db.SBDbHelper;
import chuckcoughlin.sb.assistant.utilities.SBConstants;

public class MainActivity extends AppCompatActivity {
    private static final String CLSS = "MainActivity";
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

    public MainActivity() {
        Log.i(CLSS,"Constructor ...");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
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

}
