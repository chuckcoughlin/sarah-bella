/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.os.Bundle;
import android.support.v7.widget.LinearLayoutManager;
import android.support.v7.widget.RecyclerView;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.logs.LogRecyclerAdapter;
import chuckcoughlin.sb.assistant.logs.SBLogManager;
import chuckcoughlin.sb.assistant.ros.SBApplicationStatusListener;
import chuckcoughlin.sb.assistant.ros.SBApplicationManager;

/**
 * This fragment allows perusal of the robot's activity log.
 */

public class LogsFragment extends BasicAssistantFragment implements SBApplicationStatusListener {
    private final static String CLSS = "LogFragment";
    private RecyclerView.LayoutManager layoutManager;
    private LogRecyclerAdapter adapter;
    private SBApplicationManager applicationManager;
    private RecyclerView logMessageView;
    private TextView logView;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_logs, container, false);
        TextView textView = rootView.findViewById(R.id.fragmentLogsText);
        textView.setText(R.string.fragmentLogsLabel);
        this.applicationManager = SBApplicationManager.getInstance();
        applicationManager.addListener(this);

        logMessageView = rootView.findViewById(R.id.logs_recycler_view);
        logMessageView.setHasFixedSize(false);
        LinearLayoutManager layoutManager = new LinearLayoutManager(logMessageView.getContext());
        logMessageView.setLayoutManager(layoutManager);
        adapter = new LogRecyclerAdapter();
        logMessageView.setAdapter(adapter);

        return rootView;
    }

    @Override
    public void onDestroyView() {
        Log.i(CLSS, "onDestroyView");
        SBLogManager.getInstance().removeObserver(adapter);
        applicationManager.removeListener(this);
        applicationShutdown("");
        super.onDestroyView();
    }

    // ========================================= SBApplicationStatusListener ============================
    // This may be called immediately on establishment of the listener.
    public void applicationStarted(String appName) {
        Log.i(CLSS, String.format("applicationStarted: %s ...", appName));
        SBLogManager.getInstance().addObserver(adapter);
    }

    // We don't care what the application is ...
    public void applicationShutdown(String appName) {
        Log.i(CLSS, String.format("applicationShutdown"));
        SBLogManager.getInstance().removeObserver(adapter);
    }

}
