/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant;

import android.content.Context;
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

import chuckcoughlin.sb.assistant.R;

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

        // Get the ViewPager and set it's PagerAdapter so that it can display items
        ViewPager viewPager = findViewById(R.id.viewpager);
        pagerAdapter = new MainActivityPagerAdapter(getSupportFragmentManager(),getApplicationContext());
        viewPager.setAdapter(pagerAdapter);
        Log.i(CLSS,"onCreate: complete");
    }

}
