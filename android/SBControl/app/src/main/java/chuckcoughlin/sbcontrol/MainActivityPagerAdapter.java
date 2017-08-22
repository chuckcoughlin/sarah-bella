/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sbcontrol;

import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentStatePagerAdapter;

import chuckcoughlin.sbcontrol.chuckcoughlin.sbcontrol.tab.ConfigurationFragment;
import chuckcoughlin.sbcontrol.chuckcoughlin.sbcontrol.tab.CoverFragment;
import chuckcoughlin.sbcontrol.chuckcoughlin.sbcontrol.tab.LogsFragment;
import chuckcoughlin.sbcontrol.chuckcoughlin.sbcontrol.tab.SLATFragment;

/**
 * There is a specialized page fragment for each tab position.
 * Return the appropriate fragment when requested.
 */

public class MainActivityPagerAdapter extends FragmentStatePagerAdapter {
    private final static int TAB_COUNT = 4;

    public MainActivityPagerAdapter(FragmentManager fm) {
        super(fm);
    }

    @Override
    public Fragment getItem(int position) {
        switch(position) {
            case 0:
                return new CoverFragment();
            case 1:
                return new ConfigurationFragment();
            case 2:
                return new LogsFragment();
            case 3:
                return new SLATFragment();
            default:
                return null;
        }
    }

    /**
     * @return the number of pages in our repertoire.
     */
    @Override
    public int getCount() {
        return TAB_COUNT;
    }
}
