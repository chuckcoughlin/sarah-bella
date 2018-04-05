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
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.logs.LogRecyclerAdapter;

/**
 * This fragment allows perusal of the robot's activity log.
 */

public class LogsFragment extends BasicAssistantListFragment {

    private RecyclerView.LayoutManager layoutManager;
    private LogRecyclerAdapter adapter;
    private RecyclerView logMessageView;
    private TextView logView;


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_logs, container, false);
        TextView textView = rootView.findViewById(R.id.logs_title_label);
        textView.setText(R.string.fragmentLogsLabel);

        logMessageView = rootView.findViewById(R.id.logs_recycler_view);
        logMessageView.setHasFixedSize(false);
        LinearLayoutManager layoutManager = new LinearLayoutManager(rootView.getContext());
        logMessageView.setLayoutManager(layoutManager);
        adapter = new LogRecyclerAdapter();
        logMessageView.setAdapter(adapter);

        return rootView;
    }
}
