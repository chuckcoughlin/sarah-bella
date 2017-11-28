/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.os.Bundle;
import android.support.v7.widget.LinearLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.logs.RecyclerAdapter;

/**
 * This fragment allows preusal of the robot's activity log.
 */

public class LogsFragment extends BasicAssistantFragment {

    private static final int NUM_LOG_MESSAGES = 100;  // Number of list items to display/scroll
    private RecyclerView.LayoutManager layoutManager;
    private RecyclerAdapter adapter;
    private RecyclerView logMessageView;
    private TextView logView;


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_logs, container, false);
        TextView textView = rootView.findViewById(R.id.logs_title_label);
        textView.setText(R.string.fragmentLogsLabel);

        logMessageView = rootView.findViewById(R.id.logs_recycler_view);
        // Better performance with a fixed size queue
        logMessageView.setHasFixedSize(true);
        LinearLayoutManager layoutManager = new LinearLayoutManager(rootView.getContext());
        logMessageView.setLayoutManager(layoutManager);
        logMessageView.setHasFixedSize(true);   // For now
        adapter = new RecyclerAdapter(NUM_LOG_MESSAGES);
        logMessageView.setAdapter(adapter);

        return rootView;
    }
}
