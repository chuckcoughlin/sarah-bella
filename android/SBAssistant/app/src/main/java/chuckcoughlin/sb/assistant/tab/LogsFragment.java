/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.support.v7.widget.LinearLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.tab.logs.RecyclerAdapter;

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

        logMessageView = (RecyclerView)rootView.findViewById(R.id.logs_recycler_view);
        LinearLayoutManager layoutManager = new LinearLayoutManager(rootView.getContext());
        logMessageView.setLayoutManager(layoutManager);
        logMessageView.setHasFixedSize(true);   // For now
        adapter = new RecyclerAdapter(NUM_LOG_MESSAGES);
        logMessageView.setAdapter(adapter);

        return rootView;
    }
}
