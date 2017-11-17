/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant;

import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentStatePagerAdapter;

/**
 * There is a specialized page fragment for each tab position.
 * Return the appropriate fragment when requested.
 */

public class MainActivityPagerAdapter extends FragmentStatePagerAdapter {
    private final static int TAB_COUNT = 4;
    private String tabTitles[] = new String[] { "Tab1", "Tab2", "Tab3" };
    private Context context;

    public MainActivityPagerAdapter(FragmentManager fm,Context ctx) {

        super(fm);
        this.context = ctx;
    }

    // Returns the fragment to display for that page
    @Override
    public Fragment getItem(int position) {
        switch (position) {
            case 0: // Fragment # 0 - This will show FirstFragment
                return FirstFragment.newInstance(0, "Page # 1");
            case 1: // Fragment # 0 - This will show FirstFragment different title
                return FirstFragment.newInstance(1, "Page # 2");
            case 2: // Fragment # 1 - This will show SecondFragment
                return SecondFragment.newInstance(2, "Page # 3");
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


    @Override
    public CharSequence getPageTitle(int position) {
        // Generate title based on item position
        return tabTitles[position];
    }
}
