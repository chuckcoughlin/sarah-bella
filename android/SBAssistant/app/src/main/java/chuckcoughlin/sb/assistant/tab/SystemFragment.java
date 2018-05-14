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
import android.os.BatteryManager;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;

import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.AbstractMessageListener;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBApplicationManager;
import chuckcoughlin.sb.assistant.ros.SBApplicationStatusListener;
import gpio_msgs.GPIOPin;
import gpio_msgs.GPIOPort;
import gpio_msgs.GPIOPortRequest;
import gpio_msgs.GPIOPortResponse;
import gpio_msgs.GPIOState;
import ros.android.views.BatteryLevelView;
import turtlebot3_msgs.SensorState;

/**
 * Display the current values of robot system parameters.
 */

public class SystemFragment extends BasicAssistantFragment implements SBApplicationStatusListener,
                                                                View.OnClickListener,
                                                                ServiceResponseListener<GPIOPortResponse> {
    private final static String CLSS = "SystemFragment";
    private SBApplicationManager applicationManager;
    private BatteryManager batteryManager;
    private GPIOListener gpioListener = null;
    private SensorStateListener sensorStateListener = null;
    private SystemListener systemListener = null;
    private ServiceClient<GPIOPortRequest, GPIOPortResponse> gpioInfoServiceClient = null;
    private ServiceClient<GPIOPortRequest, GPIOPortResponse> gpioGetServiceClient = null;
    private ServiceClient<GPIOPortRequest, GPIOPortResponse> gpioSetServiceClient = null;
    private View mainView = null;
    // NOTE: Pin channels are 1-based, so are off-by-one from the array index
    private final ImageView[] gpioImageViews = new ImageView[SBConstants.GPIO_PIN_COUNT];
    private final TextView[]  gpioTextViews  = new TextView[SBConstants.GPIO_PIN_COUNT];

    // Inflate the view for the fragment based on layout XML. Create fragment member objects.
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        Log.i(CLSS, "onCreateView");
        this.applicationManager = SBApplicationManager.getInstance();

        batteryManager = (BatteryManager) getActivity().getSystemService(Context.BATTERY_SERVICE);
        sensorStateListener = new SensorStateListener();
        gpioListener = new GPIOListener();
        systemListener = new SystemListener();
        applicationManager.addListener(this);

        mainView = inflater.inflate(R.layout.fragment_system, container, false);
        TextView label = mainView.findViewById(R.id.fragmentSystemText);
        label.setText(R.string.system_title);

        // Add views to the arrays. The "pinContainer" is a list of linear layouts containing text-image-image-text.
        LinearLayout pinContainer = (LinearLayout)mainView.findViewById(R.id.gpio_pins);
        int layoutCount = pinContainer.getChildCount();
        int index = 0;
        int pinNumber = 0;
        while(index<layoutCount) {
            LinearLayout pinHolder = (LinearLayout)pinContainer.getChildAt(index);
            pinNumber++;
            gpioImageViews[pinNumber-1] = (ImageView)pinHolder.getChildAt(1) ;
            gpioTextViews[pinNumber-1]  = (TextView)pinHolder.getChildAt(0) ;
            pinHolder.getChildAt(0).setTag(new Integer(pinNumber));
            pinHolder.getChildAt(1).setTag(new Integer(pinNumber));
            pinNumber++;
            gpioImageViews[pinNumber-1] = (ImageView)pinHolder.getChildAt(2) ;
            gpioTextViews[pinNumber-1]  = (TextView)pinHolder.getChildAt(3) ;
            pinHolder.getChildAt(2).setTag(new Integer(pinNumber));
            pinHolder.getChildAt(3).setTag(new Integer(pinNumber));
            index++;
        }
        Log.i(CLSS, String.format("onCreateView: %d pins configured for GPIO",pinNumber));
        return mainView;
    }
    

    @Override
    public void onDestroyView() {
        Log.i(CLSS, "onDestroyView");
        applicationManager.removeListener(this);
        applicationShutdown(SBConstants.APPLICATION_SYSTEM);
        super.onDestroyView();
    }

    // ========================================= SBApplicationStatusListener ============================
    // This may be called immediately on establishment of the listener.
    public void applicationStarted(String appName) {
        Log.i(CLSS, String.format("applicationStarted: %s ...", appName));
        if (appName.equalsIgnoreCase(SBConstants.APPLICATION_SYSTEM)) {
            ConnectedNode node = applicationManager.getCurrentApplication().getConnectedNode();
            if (node != null) {
                sensorStateListener.subscribe(node, "/sensor_state_throttle");
                gpioListener.subscribe(node,    "/gpio_msgs/values");
                systemListener.subscribe(node,  "/sb_system");
                new Thread(new Runnable(){
                    public void run() {
                        try {
                            Thread.sleep(2000);
                            gpioGetServiceClient = node.newServiceClient("/sb_serve_gpio_get", GPIOPort._TYPE);
                            Log.i(CLSS, String.format("gpioGetClient isConnected (%s)",gpioGetServiceClient.isConnected()?"TRUE":"FALSE"));
                            gpioSetServiceClient = node.newServiceClient("/sb_serve_gpio_set", GPIOPort._TYPE);
                            Log.i(CLSS, String.format("gpioSetClient isConnected (%s)",gpioSetServiceClient.isConnected()?"TRUE":"FALSE"));
                            gpioInfoServiceClient = node.newServiceClient("/sb_serve_gpio_info", GPIOPort._TYPE);
                            Log.i(CLSS, String.format("gpioInfoClient isConnected (%s)",gpioInfoServiceClient.isConnected()?"TRUE":"FALSE"));
                            checkGPIOConfiguration();
                        }
                        catch( ServiceNotFoundException snfe ) {
                            Log.e(CLSS, String.format("Exception while creating service client (%s)",snfe.getLocalizedMessage()));
                        }
                        catch (XmlRpcTimeoutException tex) {
                            // With Bluetooth - UnresolvedAddressException creating service client.
                            Log.e(CLSS, "Exception while creating service client");
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

    public void applicationShutdown(String appName) {
        if (appName.equalsIgnoreCase(SBConstants.APPLICATION_SYSTEM)) {
            Log.i(CLSS, String.format("applicationShutdown"));
            sensorStateListener.shutdown();
            gpioListener.shutdown();
            systemListener.shutdown();
            gpioGetServiceClient = null;
            gpioSetServiceClient = null;
            gpioInfoServiceClient = null;
        }
    }

    // ========================================= MessageListeners ============================
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
         * Receive a GPIOState message. The message contains all pins for which
         * values have changed. Simply update the pin value.
         * @param state the GPIOState message
         */
        @Override
        public void onNewMessage(GPIOState state) {
            //Log.i(CLSS, String.format("Got a Message - GPIOState"));
            Activity mainActivity = getActivity();
            if (mainActivity == null) {
                Log.i(CLSS, String.format("GPIOListener: Main Activity no longer available"));
                return;
            }
            mainActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    for( GPIOPin pin:state.getPins()) {
                        Log.i(CLSS, String.format("GPIOListener: State of %d is %s",(int)pin.getChannel(),(pin.getValue()?"TRUE":"FALSE")));
                        configureView(pin.getChannel(),pin.getMode(),pin.getLabel(),pin.getValue());
                    }
                }
            });
        }
    }

    public class SensorStateListener extends AbstractMessageListener<SensorState> {
        public SensorStateListener() {
            super(SensorState._TYPE);
        }

        @Override
        public void onNewMessage(SensorState bs) {
            //Log.i(CLSS, String.format("Got a Sensor Message - battery = %2.2f", bs.getBattery()/10));
            Activity mainActivity = getActivity();
            if (mainActivity == null) {
                Log.i(CLSS, String.format("SensorStateListener: Main Activity no longer available"));
                return;
            }
            mainActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    BatteryLevelView robotBattery = (BatteryLevelView) mainView.findViewById(R.id.robot_battery);
                    // A battery voltage less than 11 should trigger a shutdown.
                    robotBattery.setBatteryPercent(100.* (bs.getBattery()-11.0));
                    TextView tv = (TextView) mainView.findViewById(R.id.robot_battery_state);
                    tv.setText(String.valueOf(bs.getBattery()));
                }
            });
        }
    }
    private class SystemListener extends AbstractMessageListener<system_check.System> {
        public SystemListener() {
            super(system_check.System._TYPE);
        }

        @Override
        public void onNewMessage(system_check.System system) {
            //Log.i(CLSS, String.format("Got a System Message - CPU Usage = %2.2f", system.getCpuPercent()));
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
    // Use this method to simulate a toggle button when an image is clicked. The view tag is the pin number.
    // The only pin modes with click listeners are IN
    public void onClick(View view) {
        Log.i(CLSS,String.format("Received a click view is %s for pin %s",(view.isSelected()?"selected":"not selected"),
                (view.getTag()==null?0:view.getTag().toString())));
        byte channel = ((Integer)view.getTag()).byteValue();
        GPIOPortRequest request = gpioGetServiceClient.newMessage();
        request.setChannel(channel);
        if( gpioGetServiceClient!=null ) {
            view.setSelected(!view.isSelected());
            if (view.isSelected()) {
                view.setBackgroundResource(R.drawable.border_false);
                request.setValue(true);
            }
            else {
                view.setBackgroundResource(R.drawable.border_true);
                request.setValue(false);
            }
            gpioGetServiceClient.call(request, this);
        }
    }

    // =============================================== ServiceResponseListener =================================================
    // Use this message for all service responses. The response should contain enough info that
    // we can figure
    //            Log.i(CLSS, String.format("GPIOPortResponout what to do.
    @Override
    public void onSuccess(GPIOPortResponse response) {
        Activity mainActivity = getActivity();
        if (mainActivity == null) {
            Log.i(CLSS, String.format("Main Activity no longer available"));
            return;
        }
        Log.i(CLSS, String.format("SUCCESS: GPIOPortResponse pin %d (%s): %s:%s:%s",response.getChannel(),
                response.getMode(),response.getLabel(),(response.getValue()?"true":"false"),response.getMsg()));
        if( !response.getLabel().isEmpty() ) {
            mainActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    configureView(response.getChannel(),response.getMode(),response.getLabel(),response.getValue());
                }
            });
        }
        else if(response.getMode().equalsIgnoreCase("IN")) {

        }
    }

    @Override
    public void onFailure(RemoteException re) {
        Log.w(CLSS,String.format("Exception returned from service request (%s)",re.getLocalizedMessage()));
    }


    // ========================================= Helper Functions ============================
    // Use this method to query the robot for a GPIO pin configurations
    private void checkGPIOConfiguration() {;
        for( ImageView view:gpioImageViews ) {
            byte channel = ((Integer)view.getTag()).byteValue();
            Log.i(CLSS,String.format("Checking configuration for pin %d",channel));
            GPIOPortRequest request = gpioInfoServiceClient.newMessage();
            request.setChannel(channel);
            if( gpioInfoServiceClient!=null ) {
                request.setValue(true);
                gpioInfoServiceClient.call(request, this);
            }
         }
    }

    // Configure the GPIO image view both for the GPIO mode and value.
    private void configureView(int pinNumber,String mode,String label,boolean value) {
        if( pinNumber<=SBConstants.GPIO_PIN_COUNT ) {
            TextView  tv = gpioTextViews[pinNumber-1];
            ImageView iv =  gpioImageViews[pinNumber-1];
            if( tv==null || iv==null ) {
                Log.i(CLSS,String.format("Unable to find GPIO image or view for pin %d",pinNumber));
                return;
            }

            // If there is a label, then configure the image with the correct icon.
            if( !label.isEmpty() ) {
                tv.setText(label);
                iv.setOnClickListener(null);
                iv.setVisibility(View.VISIBLE);
                if (mode.equalsIgnoreCase("IN")) {
                    iv.setImageResource(R.drawable.ball_gray);
                }
                else if (mode.equalsIgnoreCase("OUT")) {
                    if (value) {
                        iv.setImageResource(R.drawable.ball_red);
                    }
                    else {
                        iv.setImageResource(R.drawable.ball_green);
                    }
                    Log.i(CLSS,String.format("Set click listener for pin %d",pinNumber));
                    iv.setOnClickListener(this);
                }
                else if (mode.equalsIgnoreCase("GND")) {
                    iv.setImageResource(R.drawable.ground);
                }
                else if (mode.equalsIgnoreCase("I2C")) {
                    iv.setImageResource(R.drawable.ball_blue);
                }
                else if (mode.equalsIgnoreCase("HWE")) {
                    iv.setImageResource(R.drawable.ball_blue);
                }
                else if (mode.equalsIgnoreCase("PWR")) {
                    iv.setImageResource(R.drawable.flash);
                }
                else if (mode.equalsIgnoreCase("SER")) {
                    iv.setImageResource(R.drawable.ball_blue);
                }
                else if (mode.equalsIgnoreCase("SPI")) {
                    iv.setImageResource(R.drawable.ball_blue);
                }
                else if (mode.equalsIgnoreCase("UNK")) {
                    iv.setImageResource(R.drawable.ball_yellow);
                }
                else {
                    Log.w(CLSS,String.format("GPIO port %d response has unrecognized mode (%s)",pinNumber,mode));
                }
            }

            // For IN and OUT pins, we mark the value
            if( mode.equalsIgnoreCase("IN") )  {
                if( value ) {
                    iv.setBackgroundResource(R.drawable.border_true);
                }
                else {
                    iv.setBackgroundResource(R.drawable.border_false);
                }
            }
            else if( mode.equalsIgnoreCase("OUT"))  {
                if( value ) {
                    iv.setSelected(true);
                    iv.setImageResource(R.drawable.ball_red);
                    iv.setBackgroundResource(R.drawable.border_true);
                }
                else {
                    iv.setSelected(false);
                    iv.setImageResource(R.drawable.ball_green);
                    iv.setBackgroundResource(R.drawable.border_false);
                }
            }
        }
        else {
            Log.w(CLSS,String.format("GPIO port response has illegal pin number (%d)",pinNumber));
        }

    }

}
