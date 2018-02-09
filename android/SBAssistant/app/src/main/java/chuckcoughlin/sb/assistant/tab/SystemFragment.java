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

                TextView tv =  (TextView)mainView.findViewById(R.id.cpu_usage);
                tv.setText(String.valueOf(system.getCpuPercent()));

            }
        });
    }

    /*
    public class TurtlebotDashboard extends android.widget.LinearLayout implements Dashboard.DashboardInterface {
	private ImageButton modeButton;
	private ProgressBar modeWaitingSpinner;
	private BatteryLevelView robotBattery;
	private BatteryLevelView laptopBattery;

	private ConnectedNode node;
	private Subscriber<DiagnosticArray> diagnosticSubscriber;

	private boolean powerOn = false;
	private int numModeResponses;
	private int numModeErrors;

	public TurtlebotDashboard(Context context) {
		super(context);
		inflateSelf(context);
	}

	public TurtlebotDashboard(Context context, AttributeSet attrs) {
		super(context, attrs);
		inflateSelf(context);
	}

	private void inflateSelf(Context context) {
		LayoutInflater inflater = (LayoutInflater) context.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
		inflater.inflate(R.layout.turtlebot_dashboard, this);

		modeButton = (ImageButton) findViewById(R.id.mode_button);
		modeButton.setOnClickListener(new OnClickListener() {
			@Override
			public void onClick(View v) {
				onModeButtonClicked();
			}
		});

		modeWaitingSpinner = (ProgressBar) findViewById(R.id.mode_waiting_spinner);
		modeWaitingSpinner.setIndeterminate(true);
		modeWaitingSpinner.setVisibility(View.GONE);

		robotBattery = (BatteryLevelView) findViewById(R.id.robot_battery);
		laptopBattery = (BatteryLevelView) findViewById(R.id.laptop_battery);
	}


	 * Set the ROS Node to use to get status data and connect it up. Disconnects
	 * the previous node if there was one.
	 *
	 * @throws RosException

    @Override
    public void start(ConnectedNode node) throws RosException {
        stop();
        this.node = node;
        try {
            diagnosticSubscriber = node.newSubscriber("diagnostics_agg", "diagnostic_msgs/DiagnosticArray");
            diagnosticSubscriber.addMessageListener(new MessageListener<DiagnosticArray>() {
                @Override
                public void onNewMessage(final DiagnosticArray msg) {
                    TurtlebotDashboard.this.post(new Runnable() {
                        @Override
                        public void run() {
                            TurtlebotDashboard.this.handleDiagnosticArray(msg);
                        }
                    });
                }
            });

            NameResolver resolver = node.getResolver().newChild(GraphName.of("/create_node"));
        } catch(Exception ex) {
            this.node = null;
            throw (new RosException(ex));
        }
    }

    @Override
    public void stop() {
        if(diagnosticSubscriber != null) {
            diagnosticSubscriber.shutdown();
        }
        diagnosticSubscriber = null;
        node = null;
    }


     * Populate view with new diagnostic data. This must be called in the UI
     * thread.

    private void handleDiagnosticArray(DiagnosticArray msg) {
        String mode = null;
        for(DiagnosticStatus status : msg.getStatus()) {
            if(status.getName().equals("/Power System/Battery")) {
                populateBatteryFromStatus(robotBattery, status);
            }
            if(status.getName().equals("/Power System/Laptop Battery")) {
                populateBatteryFromStatus(laptopBattery, status);
            }
            if(status.getName().equals("/Mode/Operating Mode")) {
                mode = status.getMessage();
            }
        }
        showMode(mode);
    }

    private void onModeButtonClicked() {
        powerOn = !powerOn;

        create_node.SetTurtlebotModeRequest modeRequest = node.getTopicMessageFactory().newFromType(create_node.SetTurtlebotModeRequest._TYPE);
        create_node.SetDigitalOutputsRequest setDigOutRequest = node.getTopicMessageFactory().newFromType(create_node.SetDigitalOutputsRequest._TYPE);

        setDigOutRequest.setDigitalOut1((byte) 0);
        setDigOutRequest.setDigitalOut2((byte) 0);
        if(powerOn) {
            modeRequest.setMode(create_node.TurtlebotSensorState.OI_MODE_FULL);
            setDigOutRequest.setDigitalOut0((byte) 1); // main breaker on
        } else {
            modeRequest.setMode(create_node.TurtlebotSensorState.OI_MODE_PASSIVE);
            setDigOutRequest.setDigitalOut0((byte) 0); // main breaker off
        }

        setModeWaiting(true);

        numModeResponses = 0;
        numModeErrors = 0;

        // TODO: can't I save the modeServiceClient? Causes trouble.
        try {
            ServiceClient<create_node.SetTurtlebotModeRequest, create_node.SetTurtlebotModeResponse> modeServiceClient = node.newServiceClient("turtlebot_node/set_operation_mode", "create_node/SetTurtlebotMode");
            modeServiceClient.call(modeRequest, new ServiceResponseListener<create_node.SetTurtlebotModeResponse>() {
                @Override
                public void onSuccess(create_node.SetTurtlebotModeResponse message) {
                    numModeResponses++;
                    updateModeWaiting();
                }

                @Override
                public void onFailure(RemoteException e) {
                    numModeResponses++;
                    numModeErrors++;
                    updateModeWaiting();
                }
            });
        } catch(Exception ex) {
            Toast.makeText(getContext(), "Exception in service call for set_operation_mode: " + ex.getMessage(), Toast.LENGTH_LONG).show();
            Log.i("TurtlebotDashboard", "making toast.");
        }

        try {
            ServiceClient<create_node.SetDigitalOutputsRequest, create_node.SetDigitalOutputsResponse> setDigOutServiceClient = node.newServiceClient("turtlebot_node/set_digital_outputs", "create_node/SetDigitalOutputs");
            setDigOutServiceClient.call(setDigOutRequest, new ServiceResponseListener<create_node.SetDigitalOutputsResponse>() {
                @Override
                public void onSuccess(final create_node.SetDigitalOutputsResponse msg) {
                    numModeResponses++;
                    updateModeWaiting();
                }

                @Override
                public void onFailure(RemoteException e) {
                    numModeResponses++;
                    numModeErrors++;
                    updateModeWaiting();
                }
            });
        } catch(Exception ex) {
            Toast.makeText(getContext(), "Exception in service call for set_digital_outputs: " + ex.getMessage(), Toast.LENGTH_LONG).show();
            Log.i("TurtlebotDashboard", "making toast.");
        }
    }

    private void updateModeWaiting() {
        if(numModeResponses >= 2) {
            setModeWaiting(false);
        }
    }

    private void setModeWaiting(final boolean waiting) {
        post(new Runnable() {
            @Override
            public void run() {
                modeWaitingSpinner.setVisibility(waiting ? View.VISIBLE : View.GONE);
            }
        });
    }

    private void showMode(String mode) {
        if(mode == null) {
            modeButton.setColorFilter(Color.GRAY);
        } else if(mode.equals("Full")) {
            modeButton.setColorFilter(Color.GREEN);
            powerOn = true;
        } else if(mode.equals("Safe")) {
            modeButton.setColorFilter(Color.YELLOW);
            powerOn = true;
        } else if(mode.equals("Passive")) {
            modeButton.setColorFilter(Color.RED);
            powerOn = false;
        } else {
            modeButton.setColorFilter(Color.GRAY);
            Log.w("TurtlebotDashboard", "Unknown mode string: '" + mode + "'");
        }
        setModeWaiting(false);
    }

    private void populateBatteryFromStatus(BatteryLevelView view, DiagnosticStatus status) {
        HashMap<String, String> values = keyValueArrayToMap(status.getValues());
        try {
            float percent = 100 * Float.parseFloat(values.get("Charge (Ah)")) / Float.parseFloat(values.get("Capacity (Ah)"));
            view.setBatteryPercent((int) percent);
            // TODO: set color red/yellow/green based on level (maybe with
            // level-set
            // in XML)
        } catch(NumberFormatException ex) {
            // TODO: make battery level gray
        } catch(ArithmeticException ex) {
            // TODO: make battery level gray
        } catch(NullPointerException ex) {
            // Do nothing: data wasn't there.
        }
        try {
            view.setPluggedIn(Float.parseFloat(values.get("Current (A)")) > 0);
        } catch(NumberFormatException ex) {
        } catch(ArithmeticException ex) {
        } catch(NullPointerException ex) {
        }
    }

    private HashMap<String, String> keyValueArrayToMap(List<KeyValue> list) {
        HashMap<String, String> map = new HashMap<String, String>();
        for(KeyValue kv : list) {
            map.put(kv.getKey(), kv.getValue());
        }
        return map;
    }
     */
}
