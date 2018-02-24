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

import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.Map;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.AbstractMessageListener;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBApplicationStatusListener;
import chuckcoughlin.sb.assistant.ros.SBRosApplicationManager;
import chuckcoughlin.sb.assistant.ros.SBRosManager;
import ros.android.views.BatteryLevelView;
import gpio_msgs.GPIOPin;
import gpio_msgs.GPIOState;
import sensor_msgs.BatteryState;

/**
 * Display the current values of robot system parameters.
 */

public class SystemFragment extends BasicAssistantFragment implements SBApplicationStatusListener,
                                                                      View.OnClickListener {
    private final static String CLSS = "SystemFragment";
    private final static String PUBLISH_ALL = "/gpio_msgs/publish_all";  // Flag to complete gpio
    private SBRosApplicationManager applicationManager;
    private BatteryManager batteryManager;
    private BatteryStateListener batteryListener = new BatteryStateListener();
    private GPIOListener gpioListener = new GPIOListener();
    private SystemListener systemListener = new SystemListener();
    private View mainView = null;
    private Map<View,GPIOPin> viewPinMap = new HashMap<>();

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
                systemListener.subscribe(node, "/sb_system");
                new Thread(new Runnable(){
                    public void run() {
                        try {
                            Log.i(CLSS, "Set parameter to trigger full message...");
                            Thread.sleep(5000);
                            SBRosManager rosManager = SBRosManager.getInstance();
                            String uriString = rosManager.getRobot().getRobotId().getMasterUri();
                            URI masterUri = new URI(uriString);
                            ParameterClient paramClient = new ParameterClient(new NodeIdentifier(GraphName.of("/SystemFragment"), masterUri), masterUri);
                            paramClient.setParam(GraphName.of(PUBLISH_ALL),"True");
                        }
                        catch (XmlRpcTimeoutException tex) {
                            Log.e(CLSS, "Exception while creating parameter client");
                        }
                        catch(URISyntaxException e) {
                            Log.e(CLSS, "Uri Syntax Exception while creating parameter client");
                        }
                        catch (Throwable ex) {
                            Log.e(CLSS, "Exception while creating parameter client ", ex);
                        }
                    }
                }).start();
            }
            else {
                Log.i(CLSS, String.format("applicationStarted: %s has no connected node", appName));
            }
        }
    }

    public void applicationShutdown() {
        Log.i(CLSS, String.format("applicationShutdown"));
        batteryListener.shutdown();
        gpioListener.shutdown();
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

    /**
     * This an infrequent message. Use it to configure the UI.
     * gpio_msgs
     */
    private class GPIOListener extends AbstractMessageListener<GPIOState> {
        public GPIOListener() {
            super(GPIOState._TYPE);
        }

        /**
         * Receive a GPIOState message. If the message contains all pins, then re-configure
         * the path, otherwise simply update the pin value.
         * @param state the GPIOState message
         */
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
                    for( GPIOPin pin:state.getPins()) {
                        configureView(mainView,pin);
                    }
                }
            });
        }

    }
    private void configureView(View parent, GPIOPin pin) {
        Resources res = parent.getResources();
        int id = res.getIdentifier(String.format("pin%d_label", pin.getChannel()), "id", getContext().getPackageName());
        TextView tv = (TextView) parent.findViewById(id);
        id = res.getIdentifier(String.format("pin%d_image", pin.getChannel()), "id", getContext().getPackageName());
        ImageView iv = (ImageView) mainView.findViewById(id);
        if( tv==null || iv==null ) {
            Log.i(CLSS,String.format("Unable to find GPIO image or view for pin %d",pin.getChannel()));
            return;
        }
        tv.setText(pin.getLabel());
        viewPinMap.put(iv,pin);
        iv.setOnClickListener(null);
        if (pin.getMode().equals("IN")) {
            iv.setImageResource(R.drawable.ball_yellow);
            iv.setOnClickListener(this);
        }
        else if (pin.getMode().equals("OUT")) {
            if (pin.getValue()) {
                iv.setImageResource(R.drawable.ball_red);
            } else {
                iv.setImageResource(R.drawable.ball_green);
            }
        }
        else if (pin.getMode().equals("PWR")) {
            iv.setImageResource(R.drawable.flash);
        }
        else if (pin.getMode().equals("GND")) {
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
    // =============================================== ClickListener =================================================
    // Use this method to simulate a toggle button when an image is clicked
    public void onClick(View view) {
        GPIOPin pin = viewPinMap.get(view);
        Log.i(CLSS,String.format("Received a click view is %s for pin %d",(view.isSelected()?"selected":"not selected"),(pin==null?0:pin.getChannel())));
        view.setSelected(!view.isSelected());
    }
}
