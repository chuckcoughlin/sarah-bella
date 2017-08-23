/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sbcontrol.tab;

import android.support.v4.app.Fragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import chuckcoughlin.sbcontrol.MainActivity;
import chuckcoughlin.sbcontrol.R;


/**
 * This fragment of the main activity simply shows an image of the robot.
 */

public class CoverFragment extends Fragment {

    public CoverFragment() {
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_cover, container, false);
        TextView textView = (TextView) rootView.findViewById(R.id.cover_title_label);
        textView.setText(R.string.cover_title);
        return rootView;
    }
}
