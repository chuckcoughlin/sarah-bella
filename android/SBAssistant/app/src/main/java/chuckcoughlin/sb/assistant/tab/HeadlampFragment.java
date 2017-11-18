/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.support.v4.app.Fragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import chuckcoughlin.sb.assistant.R;


/**
 * This fragment shows the status and allows control of the robot headlamp.
 */

public class HeadlampFragment extends BasicAssistantFragment {

    // Inflate the view for the fragment based on layout XML
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_headlamp, container, false);
        TextView label = (TextView) view.findViewById(R.id.fragmentHeadlampText);
        label.setText("TODO");
        return view;
    }
}
