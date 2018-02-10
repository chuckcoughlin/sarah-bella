/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 *  Based on TurtlebotDashboard.
 *      Software License Agreement (BSD License)
 *       Copyright (c) 2011, Willow Garage, Inc.
 *       All rights reserved.
 */

package chuckcoughlin.sb.assistant.tab;

import android.app.Activity;
import android.content.Context;
import android.os.BatteryManager;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.db.SBDbManager;
import chuckcoughlin.sb.assistant.ros.SBApplicationStatusListener;
import chuckcoughlin.sb.assistant.ros.SBRosApplicationManager;
import chuckcoughlin.sb.assistant.ros.SBRosManager;
import ros.android.views.BatteryLevelView;

/**
 * Display the current values of robot system parameters.
 */

public class SystemFragment extends BasicAssistantFragment implements SBApplicationStatusListener,
                                                                MessageListener<system_check.System> {
    private final static String CLSS = "SystemFragment";
    private SBRosApplicationManager applicationManager;
    private BatteryManager batteryManager;
    Subscriber<system_check.System> subscriber = null;
    private View mainView = null;

    // Inflate the view for the fragment based on layout XML
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        Log.i(CLSS, "onCreateView");
        this.applicationManager = SBRosApplicationManager.getInstance();
        applicationManager.addListener(this);
        batteryManager = (BatteryManager)getActivity().getSystemService(Context.BATTERY_SERVICE);

        mainView = inflater.inflate(R.layout.fragment_system, container, false);
        TextView label = mainView.findViewById(R.id.fragmentSystemText);
        label.setText(R.string.system_title);
        return mainView;
    }

    @Override
    public void onDestroyView() {
        Log.i(CLSS, "onDestroyView");
        applicationManager.removeListener(this);
        subscriber.removeMessageListener(this);
        super.onDestroyView();
    }

    // ========================================= SBApplicationStatusListener ============================
    public void applicationStarted(String appName) {
        Log.i(CLSS, String.format("applicationStarted: %s ...",appName));
        if(appName.equalsIgnoreCase(SBConstants.APPLICATION_SYSTEM)) {
            ConnectedNode node = applicationManager.getApplication().getConnectedNode();
            if( node!=null ) {
                subscriber = node.newSubscriber("/sb_system", system_check.System._TYPE);
                subscriber.addMessageListener(this);
            }
            else {
                Log.i(CLSS, String.format("applicationStarted: %s has no connected node",appName));
            }
        }
    }
    public void applicationShutdown() {
        Log.i(CLSS, String.format("applicationShutdown"));
        subscriber.removeMessageListener(this);
    }
    // ========================================= MessageListener ============================
    @Override
    public void onNewMessage(system_check.System system) {
        Log.i(CLSS,String.format("Got a System Message - CPU Usage = %2.2f",system.getCpuPercent()));
        Activity mainActivity = getActivity();
        if( mainActivity==null ) {
            Log.i(CLSS, String.format("Main Activity no longer available"));
            return;
        }
        mainActivity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                BatteryLevelView tabletBattery = (BatteryLevelView) mainView.findViewById(R.id.tablet_battery);
                tabletBattery.setBatteryPercent(batteryManager.getIntProperty(BatteryManager.BATTERY_PROPERTY_CAPACITY));

                TextView tv =  (TextView)mainView.findViewById(R.id.hostname);
                tv.setText(system.getHostname());
                tv =  (TextView)mainView.findViewById(R.id.ip_address);
                tv.setText(system.getIpAddress());
                tv =  (TextView)mainView.findViewById(R.id.cpu_usage);
                tv.setText(String.valueOf(system.getCpuPercent()));
                tv =  (TextView)mainView.findViewById(R.id.memory_usage);
                tv.setText(String.valueOf(system.getMemoryPercentUsed()));
                tv =  (TextView)mainView.findViewById(R.id.free_memory);
                tv.setText(String.valueOf(system.getFreeMemoryBytes()));
                tv =  (TextView)mainView.findViewById(R.id.swap_memory);
                tv.setText(String.valueOf(system.getSwapMemoryPercentUsed()));
                tv =  (TextView)mainView.findViewById(R.id.disk_usage);
                tv.setText(String.valueOf(system.getDiskPercentUsed()));
                tv =  (TextView)mainView.findViewById(R.id.packets_sent);
                tv.setText(String.valueOf(system.getPacketsSent()));
                tv =  (TextView)mainView.findViewById(R.id.packets_received);
                tv.setText(String.valueOf(system.getPacketsReceived()));
                tv =  (TextView)mainView.findViewById(R.id.in_packets_dropped);
                tv.setText(String.valueOf(system.getInPacketsDropped()));
                tv =  (TextView)mainView.findViewById(R.id.out_packets_dropped);
                tv.setText(String.valueOf(system.getOutPacketsDropped()));
            }
        });
    }
}
