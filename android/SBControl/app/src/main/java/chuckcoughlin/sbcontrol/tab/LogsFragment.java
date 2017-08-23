/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sbcontrol.tab;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.support.v7.widget.RecyclerView;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import chuckcoughlin.sbcontrol.R;
import chuckcoughlin.sbcontrol.tab.logs.RecyclerAdapter;

/**
 * This fragment allows preusal of the robot's activity log.
 */

public class LogsFragment extends Fragment {

    private static final int NUM_LOG_MESSAGES = 100;  // Number of list items to display/scroll
    private RecyclerAdapter adapter;
    private RecyclerView logMessageView;
    private TextView logView;

    public LogsFragment() {
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_logs, container, false);
        TextView textView = (TextView) rootView.findViewById(R.id.logs_title_label);
        textView.setText(R.string.logs_title);
        return rootView;
    }
}
