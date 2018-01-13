/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.content.Context;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ListAdapter;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.List;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.dialog.SBBasicDialogFragment;
import chuckcoughlin.sb.assistant.dialog.SBDialogCallbackHandler;
import chuckcoughlin.sb.assistant.dialog.SBRobotCreateDialog;
import chuckcoughlin.sb.assistant.dialog.SBWarningDialog;
import chuckcoughlin.sb.assistant.ros.SBRosApplicationManager;
import chuckcoughlin.sb.assistant.ros.SBRosManager;
import ros.android.appmanager.SBRobotConnectionHandler;
import ros.android.appmanager.WifiChecker;
import ros.android.appmanager.MasterChecker;
import ros.android.util.RobotApplication;
import ros.android.util.RobotDescription;

import static android.content.Context.WIFI_SERVICE;
import static chuckcoughlin.sb.assistant.common.SBConstants.DIALOG_TRANSACTION_KEY;

/**
 * Display the current robot. Provide for its creation if it doesn't exist and for
 * its editing if it does. We can connect, if needed and then start or stop applications.
 * Lifecycle methods are presented here in chronological order.
 */
public class DiscoveryFragment extends BasicAssistantListFragment implements SBDialogCallbackHandler, SBRobotConnectionHandler {
    private final static String CLSS = "DiscoveryFragment";
    private SBRosManager rosManager;
    private SBRosApplicationManager applicationManager;
    private View contentView = null;


    // Called when the fragment's instance initializes
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(CLSS, "DiscoveryFragment.onCreate");
        super.onCreate(savedInstanceState);
        this.rosManager = SBRosManager.getInstance();
        this.applicationManager = SBRosApplicationManager.getInstance();
    }

    // Called to have the fragment instantiate its user interface view.
    // Inflate the view for the fragment based on layout XML. Populate
    // the text fields from the database.
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        Log.i(CLSS, "DiscoveryFragment.onCreateView");
        this.contentView = inflater.inflate(R.layout.fragment_discovery, container, false);
        TextView textView = contentView.findViewById(R.id.fragmentDiscoveryText);
        textView.setText(R.string.fragmentDiscoveryLabel);

        Button button = (Button) contentView.findViewById(R.id.defineButton);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                defineRobotClicked();
            }
        });

        button = (Button) contentView.findViewById(R.id.connectButton);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                connectRobotClicked();
            }
        });
        button = (Button) contentView.findViewById(R.id.startButton);
        ;
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startApplicationClicked();
            }
        });
        updateUI();
        return contentView;
    }

    // Executes after onCreateView()
    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        Log.i(CLSS, "DiscoveryFragment.onViewCreated");
    }

    // The host activity has been created.
    @Override
    public void onActivityCreated(Bundle savedInstanceState) {
        super.onActivityCreated(savedInstanceState);
        Log.i(CLSS, "DiscoveryFragment.onAxtivityCreated");
        RobotApplicationsAdapter adapter = new RobotApplicationsAdapter(getContext(), new ArrayList<>());
        setListAdapter(adapter);
        getListView().setItemsCanFocus(true);

        RobotDescription robot = rosManager.getRobot();
        if (robot != null) {
            List<RobotApplication> applicationList = SBRosApplicationManager.getInstance().getApplications();
            Log.i(CLSS, String.format("onActivityCreated: will display %d applications for %s", applicationList.size(), rosManager.getRobot().getRobotName()));
            adapter.clear();
            adapter.addAll(applicationList);
        }
    }

    // The fragment is visible
    @Override
    public void onStart() {
        super.onStart();
    }

    // The Fragment is visible and inter-actable
    @Override
    public void onResume() {
        super.onResume();
    }

    // Called when user leaves the current fragment or fragment is no longer inter-actable
    @Override
    public void onPause() {
        super.onPause();
    }

    // The fragment is going to be stopped
    @Override
    public void onStop() {
        super.onStop();
    }

    // Cleanup resources created in onCreateView()
    @Override
    public void onDestroyView() {
        super.onDestroyView();
        this.contentView = null;
    }

    // Execute any final cleanup for the fragment's state
    @Override
    public void onDestroy() {
        super.onDestroy();
    }

    // The fragment has been disassociated from its hosting activity
    @Override
    public void onDetach() {
        super.onDetach();
    }

    // =========================================== Checker Callback ====================================
    @Override
    public void handleConnectionError(String reason) {
        Log.w(CLSS, "handleConnectionError: " + reason);
        rosManager.setConnectionStatus(RobotDescription.CONNECTION_STATUS_UNCONNECTED);
        SBWarningDialog warning = SBWarningDialog.newInstance("Connection Error", reason);
        warning.show(getActivity().getFragmentManager(), DIALOG_TRANSACTION_KEY);
    }

    @Override
    public void handleNetworkError(String reason) {
        Log.w(CLSS, "handleNetworkError: " + reason);
        rosManager.setConnectionStatus(RobotDescription.CONNECTION_STATUS_UNAVAILABLE);
        SBWarningDialog warning = SBWarningDialog.newInstance("Network Error", reason);
        warning.show(getActivity().getFragmentManager(), DIALOG_TRANSACTION_KEY);
    }

    @Override
    public boolean handleWifiReconnection(String SSID, String networkName) {
        return true;
    }

    // The application name is a global parameter of the robot.
    @Override
    public void receiveApplication(String appName) {
        Log.w(CLSS, "receiveApplication: " + appName);
    }

    // Once the robot is connected, interrogate it for applications
    @Override
    public void receiveConnection(RobotDescription robot) {
        Log.w(CLSS, "receiveConnection: SUCCESS!");
        rosManager.updateRobot(robot);
        rosManager.setConnectionStatus(RobotDescription.CONNECTION_STATUS_CONNECTED);
    }

    // Once the wifi is connected, check the robot, itself
    @Override
    public void receiveWifiConnection() {
        MasterChecker checker = new MasterChecker(this);
        RobotDescription robot = rosManager.getRobot();
        if (robot == null) return;
        checker.beginChecking(robot.getRobotId());
        /*
        try {
            appManager = createAppManagerCb(node, robotDescription);
        } catch(RosException e) {
            Log.e("RosAndroid", "ros init failed", e);
            appManager = null;
        } catch(XmlRpcTimeoutException e) {
            Log.e("RosAndroid", "ros init failed", e);
            appManager = null;
        } catch(AppManagerNotAvailableException e) {
            Log.e("RosAndroid", "ros init failed", e);
            appManager = null;
        }
        if(appManager != null && startApplication) {
            appManager.addTerminationCallback(robotAppName, new ApplicationManager.TerminationCallback() {
                @Override
                public void onAppTermination() {
                    RosAppActivity.this.onAppTerminate();
                }
            });
        }
        try {
            // Start up the application on the robot and start the dashboard.
            dashboard.start(node);
            if(startApplication && false) {
                applicationStarted = false;
                startApp();
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        if(progress != null) {
                            progress.dismiss();
                        }
                        progress = ProgressDialog.show(RosAppActivity.this, "Starting...", "Starting application...", true, false);
                        progress.setProgressStyle(ProgressDialog.STYLE_SPINNER);
                    }
                });
                try {
                    while(!applicationStarted) {
                        Thread.sleep(100);
                    }
                } catch(java.lang.InterruptedException e) {
                    Log.i("RosAndroid", "Caught interrupted exception while spinning");
                }
                unOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        if(progress != null) {
                            progress.dismiss();
                        }
                        progress = null;
                    }
                });
            } else {
                Log.i("RosAndroid", "Not starting application");
            }
        } catch(Exception ex) {
            Log.e("$rootclass", "Init error: " + ex.toString());
            safeToastStatus("Failed: " + ex.getMessage());
        }
        */

    }

    // =========================================== Dialog Callback =====================================
    @Override
    public void handleDialogResult(SBBasicDialogFragment dfrag) {
        Log.i(CLSS, String.format("handleDialogResults for %s", dfrag.getDialogType()));
        if (dfrag.getDialogType().equalsIgnoreCase(SBRobotCreateDialog.CLSS)) {
            String btn = dfrag.getSelectedButton();
            Log.i(CLSS, String.format("handleDialogResults clicked on %s", btn));
            if (btn.equalsIgnoreCase(SBConstants.DIALOG_BUTTON_ADD)) {
                String msg = dfrag.getErrorMessage();
                Log.i(CLSS, String.format("handleDialogResults error is %s", msg));
                RobotDescription robot = (RobotDescription) dfrag.getPayload();

                if (msg != null && !msg.isEmpty()) {
                    final Toast toast = Toast.makeText(getActivity(), msg, Toast.LENGTH_LONG);
                    toast.show();
                } else if (robot != null) {
                    if (rosManager.getRobot() == null) rosManager.createRobot(robot);
                    else {
                        rosManager.updateRobot(robot);
                    }

                    List<RobotApplication> apps = applicationManager.getApplications();
                    RobotApplicationsAdapter adapter = (RobotApplicationsAdapter) getListAdapter();
                    adapter.clear();
                    for (RobotApplication app : apps) {
                        adapter.add(app);
                        Log.i(CLSS, String.format("handleDialogResults added %s - now have %d", app.getApplicationName(), applicationManager.getApplicationCount()));
                    }
                    adapter.notifyDataSetInvalidated();

                }
                updateUI();
            }
        }
    }


    //======================================== Array Adapter ======================================
    public class RobotApplicationsAdapter extends ArrayAdapter<RobotApplication> implements ListAdapter {

        public RobotApplicationsAdapter(Context context, List<RobotApplication> values) {
            super(context, R.layout.ros_application_item, values);
        }

        @Override
        public long getItemId(int position) {
            return getItem(position).hashCode();
        }

        @Override
        public View getView(int position, View convertView, ViewGroup parent) {
            Log.i(CLSS, String.format("RobotApplicationsAdapter.getView position =  %d", position));
            // Get the data item for this position
            RobotApplication app = getItem(position);
            // Check if an existing view is being reused, otherwise inflate the view
            if (convertView == null) {
                Log.i(CLSS, String.format("RobotApplicationsAdapter.getView convertView was null"));
                convertView = LayoutInflater.from(getContext()).inflate(R.layout.ros_application_item, parent, false);
            }

            // Lookup view for data population
            TextView nameView = (TextView) convertView.findViewById(R.id.name);
            TextView uriView = (TextView) convertView.findViewById(R.id.uri);
            TextView statusView = (TextView) convertView.findViewById(R.id.status);

            // Populate the data into the template view using the data object
            nameView.setText(app.getApplicationName());
            //uriView.setText(description.getRobotId().getMasterUri());
            //statusView.setText(description.getConnectionStatus());
            /*
            listView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
                @Override
                public void onItemClick(AdapterView<?> parent, View v, int position, long id) {
                    choose(position);
                }
            });

            Log.i(CLSS,String.format("RobotApplicationsAdapter.getView set %s(%s) = %s",description.getRobotName(),description.getRobotId(),description.getConnectionStatus()));
            int index = 0;
            int count = getCount();
            while( index<count ) {
                RobotDescription robot = getItem(index);
                if( robot != null && robot.equals( rosManager.getCurrentRobot() )) {
                    Log.i(CLSS, "Highlighting index " + index);
                    listView.setItemChecked(index, true);
                    break;
                }
                index++;
            }
            */
            // Return the completed view to render on screen
            return convertView;
        }

        private void choose(int position) {
            //SBRosApplicationManager.getInstance().setCurrentApplication(position);
        }

    }
    //======================================== Update the UI ======================================

    /**
     * Keep the views in-sync with the model state
     */
    private void updateUI() {
        RobotDescription robot = rosManager.getRobot();
        Button button = (Button) contentView.findViewById(R.id.defineButton);
        if (robot == null) button.setText(R.string.discoveryButtonDefine);
        else button.setText((R.string.discoveryButtonEdit));


        button = (Button) contentView.findViewById(R.id.connectButton);
        button.setEnabled(robot != null);
        if (robot == null || !robot.getConnectionStatus().equalsIgnoreCase(RobotDescription.CONNECTION_STATUS_CONNECTED))
            button.setText(R.string.discoveryButtonConnect);
        else button.setText((R.string.discoveryButtonDisconnect));

        button = (Button) contentView.findViewById(R.id.startButton);
        button.setEnabled(robot != null && applicationManager.getCurrentApplication() != null);

        ImageView iview = (ImageView) contentView.findViewById(R.id.robot_icon);
        if (robot == null) iview.setVisibility(View.INVISIBLE);
        else iview.setVisibility(View.VISIBLE);

        iview = (ImageView) contentView.findViewById(R.id.error_icon);
        if (robot == null) iview.setVisibility(View.INVISIBLE);
        else iview.setVisibility(View.VISIBLE);

        ProgressBar bar = (ProgressBar) contentView.findViewById(R.id.progress_circle);
        if (robot == null || !robot.getConnectionStatus().equalsIgnoreCase(RobotDescription.CONNECTION_STATUS_CONNECTING))
            bar.setVisibility(View.INVISIBLE);
        else {
            bar.setVisibility(View.VISIBLE);
            bar.setIndeterminate(true);
        }
        TextView tview = (TextView) contentView.findViewById(R.id.robot_name);
        if (robot == null) tview.setVisibility(View.INVISIBLE);
        else {
            tview.setText(robot.getRobotName());
            tview.setVisibility(View.VISIBLE);
        }

        tview = (TextView) contentView.findViewById(R.id.master_uri);
        if (robot == null) tview.setVisibility(View.INVISIBLE);
        else {
            tview.setText(robot.getRobotId().getMasterUri());
            tview.setVisibility(View.VISIBLE);
        }

        tview = (TextView) contentView.findViewById(R.id.status);
        if (robot == null) tview.setVisibility(View.INVISIBLE);
        else {
            tview.setText(applicationManager.getApplicationStatus());
            tview.setVisibility(View.VISIBLE);
        }
    }

    //======================================== Button Callbacks ======================================
    public void defineRobotClicked() {
        Log.i(CLSS, "Add robot clicked");
        SBRobotCreateDialog addDialog = new SBRobotCreateDialog();
        addDialog.setHandler(this);
        addDialog.show(getActivity().getFragmentManager(), DIALOG_TRANSACTION_KEY);
    }

    public void connectRobotClicked() {
        Log.i(CLSS, "Connect robot clicked");
        RobotDescription robot = rosManager.getRobot();
        if (robot == null) return; // Shouldn't happen
        WifiChecker checker = new WifiChecker(this);
        checker.beginChecking(robot.getRobotId(), (WifiManager) getActivity().getApplicationContext().getSystemService(WIFI_SERVICE));

    }

    public void startApplicationClicked() {
        Log.i(CLSS, "Start/stop application clicked");
    }


}
