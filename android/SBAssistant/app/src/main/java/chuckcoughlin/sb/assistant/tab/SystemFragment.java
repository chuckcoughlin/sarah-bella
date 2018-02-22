/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * (MIT License)
 * Based on TurtlebotDashboard.
 * Software License Agreement (BSD License)
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 */

package chuckcoughlin.sb.assistant.tab;

import android.app.Activity;
import android.content.Context;
import android.content.res.Resources;
import android.os.BatteryManager;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;

import org.ros.node.ConnectedNode;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.AbstractMessageListener;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBApplicationStatusListener;
import chuckcoughlin.sb.assistant.ros.SBRosApplicationManager;
import ros.android.views.BatteryLevelView;
import gpio_msgs.GPIOPin;
import gpio_msgs.GPIOState;
import sensor_msgs.BatteryState;

/**
 * Display the current values of robot system parameters.
 */

public class SystemFragment extends BasicAssistantFragment implements SBApplicationStatusListener {
    private final static String CLSS = "SystemFragment";
    private SBRosApplicationManager applicationManager;
    private BatteryManager batteryManager;
    private BatteryStateListener batteryListener = new BatteryStateListener();
    private GPIOListener gpioListener = new GPIOListener();
    private GPIOStateListener gpioStateListener = new GPIOStateListener();
    private SystemListener systemListener = new SystemListener();
    private View mainView = null;

    // Inflate the view for the fragment based on layout XML
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        Log.i(CLSS, "onCreateView");
        this.applicationManager = SBRosApplicationManager.getInstance();
        applicationManager.addListener(this);
        batteryManager = (BatteryManager) getActivity().getSystemService(Context.BATTERY_SERVICE);

        mainView = inflater.inflate(R.layout.fragment_system, container, false);
        TextView label = mainView.findViewById(R.id.fragmentSystemText);
        label.setText(R.string.system_title);
        return mainView;
    }

    @Override
    public void onDestroyView() {
        Log.i(CLSS, "onDestroyView");
        applicationManager.removeListener(this);

        super.onDestroyView();
    }

    // ========================================= SBApplicationStatusListener ============================
    public void applicationStarted(String appName) {
        Log.i(CLSS, String.format("applicationStarted: %s ...", appName));
        if (appName.equalsIgnoreCase(SBConstants.APPLICATION_SYSTEM)) {
            ConnectedNode node = applicationManager.getApplication().getConnectedNode();
            if (node != null) {
                batteryListener.subscribe(node, "/sensor_msgs");
                gpioListener.subscribe(node, "/gpio_msgs");
                gpioStateListener.subscribe(node, "/gpio_msgs");
                systemListener.subscribe(node, "/sb_system");
            } else {
                Log.i(CLSS, String.format("applicationStarted: %s has no connected node", appName));
            }
        }
    }

    public void applicationShutdown() {
        Log.i(CLSS, String.format("applicationShutdown"));
        batteryListener.shutdown();
        gpioListener.shutdown();
        gpioStateListener.shutdown();
        systemListener.shutdown();
    }

    // ========================================= MessageListeners ============================
    public class BatteryStateListener extends AbstractMessageListener<BatteryState> {
        public BatteryStateListener() {
            super(BatteryState._TYPE);
        }

        @Override
        public void onNewMessage(BatteryState bs) {
            Log.i(CLSS, String.format("Got a Battery Message - capacity = %2.2f", bs.getCapacity()));
            Activity mainActivity = getActivity();
            if (mainActivity == null) {
                Log.i(CLSS, String.format("BatteryStateListener: Main Activity no longer available"));
                return;
            }
            mainActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    BatteryLevelView robotBattery = (BatteryLevelView) mainView.findViewById(R.id.robot_battery);
                    robotBattery.setBatteryPercent(bs.getCapacity());
                    TextView tv = (TextView) mainView.findViewById(R.id.robot_battery_state);
                    tv.setText(String.valueOf(bs.getCapacity()));
                }
            });
        }
    }

    // gpio_msgs
    private class GPIOListener extends AbstractMessageListener<GPIOPin> {
        public GPIOListener() {
            super(GPIOPin._TYPE);
        }

        @Override
        public void onNewMessage(GPIOPin pin) {
            Log.i(CLSS, String.format("Got a GPIO pin Message - channel = %d", pin.getChannel()
            ));
            Activity mainActivity = getActivity();
            if (mainActivity == null) {
                Log.i(CLSS, String.format("GPIOListener: Main Activity no longer available"));
                return;
            }
            mainActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    BatteryLevelView tabletBattery = (BatteryLevelView) mainView.findViewById(R.id.tablet_battery);
                    tabletBattery.setBatteryPercent(batteryManager.getIntProperty(BatteryManager.BATTERY_PROPERTY_CAPACITY));

                }
            });
        }
    }

    /**
     * This an infrequent message. Use it to configure the UI.
     * gpio_msgs
     */
    private class GPIOStateListener extends AbstractMessageListener<GPIOState> {
        public GPIOStateListener() {
            super(GPIOState._TYPE);
        }

        @Override
        public void onNewMessage(GPIOState state) {
            Log.i(CLSS, String.format("Got a Message - GPIOState"));
            Activity mainActivity = getActivity();
            if (mainActivity == null) {
                Log.i(CLSS, String.format("GPIOStateListener: Main Activity no longer available"));
                return;
            }
            mainActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    configureView(mainView, state.getPin1());
                    configureView(mainView, state.getPin2());
                    configureView(mainView, state.getPin3());
                    configureView(mainView, state.getPin4());
                    configureView(mainView, state.getPin5());
                    configureView(mainView, state.getPin6());
                    configureView(mainView, state.getPin7());
                    configureView(mainView, state.getPin8());
                    configureView(mainView, state.getPin9());
                    configureView(mainView, state.getPin10());
                    configureView(mainView, state.getPin11());
                    configureView(mainView, state.getPin12());
                    configureView(mainView, state.getPin13());
                    configureView(mainView, state.getPin14());
                    configureView(mainView, state.getPin15());
                    configureView(mainView, state.getPin16());
                    configureView(mainView, state.getPin17());
                    configureView(mainView, state.getPin18());
                    configureView(mainView, state.getPin19());
                    configureView(mainView, state.getPin20());
                    configureView(mainView, state.getPin21());
                    configureView(mainView, state.getPin22());
                    configureView(mainView, state.getPin23());
                    configureView(mainView, state.getPin24());
                    configureView(mainView, state.getPin25());
                    configureView(mainView, state.getPin26());
                    configureView(mainView, state.getPin27());
                    configureView(mainView, state.getPin28());
                    configureView(mainView, state.getPin29());
                    configureView(mainView, state.getPin30());
                    configureView(mainView, state.getPin31());
                    configureView(mainView, state.getPin32());
                    configureView(mainView, state.getPin33());
                    configureView(mainView, state.getPin34());
                    configureView(mainView, state.getPin35());
                    configureView(mainView, state.getPin36());
                    configureView(mainView, state.getPin37());
                    configureView(mainView, state.getPin38());
                    configureView(mainView, state.getPin39());
                    configureView(mainView, state.getPin40());
                }
            });
        }

    }
    private void configureView(View parent, GPIOPin pin) {
        Resources res = parent.getResources();
        int id = res.getIdentifier(String.format("pin%d_label", pin.getChannel()), "id", getContext().getPackageName());
        TextView tv = (TextView) parent.findViewById(id);
        tv.setText(pin.getLabel());
        id = res.getIdentifier(String.format("pin%d_image", pin.getChannel()), "id", getContext().getPackageName());
        ImageView iv = (ImageView) mainView.findViewById(id);
        if (pin.getMode().equals("IN")) {
            iv.setImageResource(R.drawable.ball_yellow);
        } else if (pin.getMode().equals("OUT")) {
            if (pin.getValue()) {
                iv.setImageResource(R.drawable.ball_red);
            } else {
                iv.setImageResource(R.drawable.ball_green);
            }
        } else if (pin.getMode().equals("PWR")) {
            iv.setImageResource(R.drawable.flash);
        } else if (pin.getMode().equals("GND")) {
            iv.setImageResource(R.drawable.ground);
        }
    }

    private class SystemListener extends AbstractMessageListener<system_check.System> {
        public SystemListener() {
            super(system_check.System._TYPE);
        }

        @Override
        public void onNewMessage(system_check.System system) {
            Log.i(CLSS, String.format("Got a System Message - CPU Usage = %2.2f", system.getCpuPercent()));
            Activity mainActivity = getActivity();
            if (mainActivity == null) {
                Log.i(CLSS, String.format("Main Activity no longer available"));
                return;
            }
            mainActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    BatteryLevelView tabletBattery = (BatteryLevelView) mainView.findViewById(R.id.tablet_battery);
                    tabletBattery.setBatteryPercent(batteryManager.getIntProperty(BatteryManager.BATTERY_PROPERTY_CAPACITY));

                    TextView tv = (TextView) mainView.findViewById(R.id.hostname);
                    tv.setText(system.getHostname());
                    tv = (TextView) mainView.findViewById(R.id.ip_address);
                    tv.setText(system.getIpAddress());
                    tv = (TextView) mainView.findViewById(R.id.cpu_usage);
                    tv.setText(String.valueOf(system.getCpuPercent()));
                    tv = (TextView) mainView.findViewById(R.id.memory_usage);
                    tv.setText(String.valueOf(system.getMemoryPercentUsed()));
                    tv = (TextView) mainView.findViewById(R.id.free_memory);
                    tv.setText(String.valueOf(system.getFreeMemoryBytes()));
                    tv = (TextView) mainView.findViewById(R.id.swap_memory);
                    tv.setText(String.valueOf(system.getSwapMemoryPercentUsed()));
                    tv = (TextView) mainView.findViewById(R.id.disk_usage);
                    tv.setText(String.valueOf(system.getDiskPercentUsed()));
                    tv = (TextView) mainView.findViewById(R.id.packets_sent);
                    tv.setText(String.valueOf(system.getPacketsSent()));
                    tv = (TextView) mainView.findViewById(R.id.packets_received);
                    tv.setText(String.valueOf(system.getPacketsReceived()));
                    tv = (TextView) mainView.findViewById(R.id.in_packets_dropped);
                    tv.setText(String.valueOf(system.getInPacketsDropped()));
                    tv = (TextView) mainView.findViewById(R.id.out_packets_dropped);
                    tv.setText(String.valueOf(system.getOutPacketsDropped()));
                }
            });
        }
    }
}
