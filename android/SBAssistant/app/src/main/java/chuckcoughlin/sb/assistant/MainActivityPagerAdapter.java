/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant;

import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentStatePagerAdapter;

import chuckcoughlin.sb.assistant.tab.AudioFragment;
import chuckcoughlin.sb.assistant.tab.BasicAssistantFragment;
import chuckcoughlin.sb.assistant.tab.CameraFragment;
import chuckcoughlin.sb.assistant.tab.CoverFragment;
import chuckcoughlin.sb.assistant.tab.HeadlampFragment;
import chuckcoughlin.sb.assistant.tab.LogsFragment;
import chuckcoughlin.sb.assistant.tab.SLAMFragment;
import chuckcoughlin.sb.assistant.tab.SystemFragment;


/**
 * There is a specialized page fragment for each tab position.
 * Return the appropriate fragment when requested.
 */

public class MainActivityPagerAdapter extends FragmentStatePagerAdapter {
    private final String tabTitles[] = new String[] { "@string/cover_title", "@string/system_title", "@string/headlamp_title",
                                                      "@string/SLAM_title","@string/audio_title","@string/camera_title","@string/log_title"};

    public MainActivityPagerAdapter(FragmentManager fm) {
        super(fm);
    }

    /**
     * Each page is a different class.
     * @param position page number
     * @return a new instance of the page.
     */
    @Override
    public Fragment getItem(int position) {
        BasicAssistantFragment frag = null;
        switch (position) {
            case 0:
                frag = new CoverFragment();
            case 1:
                frag =  new SystemFragment();
            case 2:
                frag = new HeadlampFragment();
            case 3:
                frag = new SLAMFragment();
            case 4:
                frag = new AudioFragment();
            case 5:
                frag = new CameraFragment();
            case 6:
                frag = new LogsFragment();
            default:
                ;
        }
        if( frag!=null ) {
            frag.setPageNumber(position);
            frag.setTitle(tabTitles[position]);
        }
        return frag;
    }



    /**
     * @return the number of pages in our repertoire.
     */
    @Override
    public int getCount() {
        return tabTitles.length;
    }


    @Override
    public CharSequence getPageTitle(int position) {
        return tabTitles[position];
    }
}
