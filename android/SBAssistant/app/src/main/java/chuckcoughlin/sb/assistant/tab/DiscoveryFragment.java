/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.bluetooth.BluetoothManager;
import android.content.Context;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AbsListView;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ListAdapter;
import android.widget.ListView;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import java.util.ArrayList;
import java.util.List;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.db.SBDbManager;
import chuckcoughlin.sb.assistant.dialog.SBRobotViewDialog;
import chuckcoughlin.sb.assistant.dialog.SBWarningDialog;
import chuckcoughlin.sb.assistant.ros.SBRosApplicationManager;
import chuckcoughlin.sb.assistant.ros.SBRosManager;
import ros.android.appmanager.BluetoothChecker;
import ros.android.appmanager.MasterChecker;
import ros.android.appmanager.RemoteCommand;
import ros.android.appmanager.SBRobotConnectionHandler;
import ros.android.appmanager.WifiChecker;
import ros.android.util.RobotApplication;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

import static android.content.Context.BLUETOOTH_SERVICE;
import static android.content.Context.WIFI_SERVICE;
import static chuckcoughlin.sb.assistant.common.SBConstants.DIALOG_TRANSACTION_KEY;

/**
 * Orchestrate connections to the network, the robot and the desired application.
 * Lifecycle methods are presented here in chronological order. Use the SBRosManager
 * instance to preserve connection state whenever the fragment is not displayed.
 */
public class DiscoveryFragment extends BasicAssistantListFragment implements SBRobotConnectionHandler,
                                                AdapterView.OnItemClickListener {
    private final static String CLSS = "DiscoveryFragment";
    private final static String SET_APP_COMMAND = "~/robotics/robot/bin/set_ros_application %s";
    private final static String START_APP_COMMAND = "~/robotics/robot/bin/restart_ros";
    private View contentView = null;
    private ViewGroup viewGroup = null;
    private SBDbManager dbManager = null;
    private SBRosManager rosManager = null;
    private SBRosApplicationManager applicationManager = null;
    private RemoteCommand command = null;


    // Called when the fragment's instance initializes
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(CLSS, "DiscoveryFragment.onCreate");
        super.onCreate(savedInstanceState);
    }

    // Called to have the fragment instantiate its user interface view.
    // Inflate the view for the fragment based on layout XML. Populate
    // the text fields and application list from the database.
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        Log.i(CLSS, "DiscoveryFragment.onCreateView");
        this.contentView = inflater.inflate(R.layout.fragment_discovery, container, false);
        this.viewGroup = container;
        TextView textView = contentView.findViewById(R.id.fragmentDiscoveryText);
        textView.setText(R.string.fragmentDiscoveryLabel);

        Button button = (Button) contentView.findViewById(R.id.connectButton);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                connectRobotClicked();
            }
        });
        button = (Button) contentView.findViewById(R.id.viewButton);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                viewRobotClicked();
            }
        });

        return contentView;
    }

    // Executes after onCreateView()
    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        Log.i(CLSS, "DiscoveryFragment.onViewCreated");
        dbManager = SBDbManager.getInstance();
        rosManager = SBRosManager.getInstance();
        applicationManager = SBRosApplicationManager.getInstance();
        command = new RemoteCommand(dbManager.getSetting(SBConstants.ROS_HOST),
                                    dbManager.getSetting(SBConstants.ROS_USER),
                                    dbManager.getSetting(SBConstants.ROS_USER_PASSWORD),this);
        configureListView();
    }

    // The host activity has been created.
    @Override
    public void onActivityCreated(Bundle savedInstanceState) {
        super.onActivityCreated(savedInstanceState);
        Log.i(CLSS, "DiscoveryFragment.onActivityCreated");
        // We populate the list whether or not there is a robot connection. We simply hide it as appropriate.
        configureListView();
        updateUI();
    }

    private void configureListView() {
        ListView listView = getListView();
        listView.setItemsCanFocus(true);
        listView.setChoiceMode(AbsListView.CHOICE_MODE_SINGLE);
        listView.setVisibility(View.INVISIBLE);
        listView.setEnabled(false);
        //listView.setOnItemClickListener(this);
        listView.setClickable(true);
        RobotApplicationsAdapter adapter = new RobotApplicationsAdapter(getContext(), new ArrayList<>());
        setListAdapter(adapter);
        adapter.clear();
        List<RobotApplication> applicationList = SBRosApplicationManager.getInstance().getApplications();
        Log.i(CLSS, String.format("configureListView: will display %d applications for all robots", applicationList.size()));
        adapter.addAll(applicationList);
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
        Log.i(CLSS, "DiscoveryFragment.onDestroyView");
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

    //======================================== Array Adapter ======================================
    public class RobotApplicationsAdapter extends ArrayAdapter<RobotApplication> implements ListAdapter {

        public RobotApplicationsAdapter(Context context, List<RobotApplication> values) {
            super(context, R.layout.discovery_application_item, values);
        }

        @Override
        public long getItemId(int position) {
            return getItem(position).hashCode();
        }

        // The view here is the row containing app name, description and on/off button. It appears that
        // clickable elements within the layout make the entire row unclickable. So we re-add the listener.
        @Override
        public View getView(int position, View convertView, ViewGroup parent) {
            //Log.i(CLSS, String.format("RobotApplicationsAdapter.getView position =  %d", position));
            // Get the data item for this position
            RobotApplication app = getItem(position);
            // Check if an existing view is being reused, otherwise inflate the view
            if (convertView == null) {
                //Log.i(CLSS, String.format("RobotApplicationsAdapter.getView convertView was null"));
                convertView = LayoutInflater.from(getContext()).inflate(R.layout.discovery_application_item, parent, false);
            }

            convertView.setOnClickListener(new View.OnClickListener() {
                public void onClick(View view) {
                    onItemClick(getListView(),view,position,position);
                }
            });

            // Lookup view for data population
            TextView nameView = (TextView) convertView.findViewById(R.id.application_name);
            TextView descriptionView = (TextView) convertView.findViewById(R.id.application_description);
            ToggleButton statusToggle    = convertView.findViewById(R.id.application_selector);
            statusToggle.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    startApplicationClicked();
                }
            });

            // Populate the data into the template view using the data object
            nameView.setText(app.getApplicationName());
            descriptionView.setText(app.getDescription());
            updateStatusImage(app,statusToggle);

            // Return the completed view to render on screen
            convertView.postInvalidate(0,0,convertView.getRight(),convertView.getBottom());
            return convertView;
        }
    }

    /**
     * Update the application status icon at the indicated position in the list. Use this version
     * where the application template is known. Run this on the UI Thread.
     *
     * @param app the current application
     * @param toggle button that shows the application state
     */
    private void updateStatusImage(final RobotApplication app,ToggleButton toggle ) {
        Log.i(CLSS, String.format("updateStatusImage: for %s (%s)",app.getApplicationName(),
                (applicationManager.getApplication()==null?null:applicationManager.getApplication().getExecutionStatus())));

        getActivity().runOnUiThread(new Runnable() {
            public void run() {
                toggle.setVisibility(View.INVISIBLE);
                if (applicationManager.getApplication() != null && app.getApplicationName().equalsIgnoreCase(applicationManager.getApplication().getApplicationName())) {
                    RobotApplication rapp = applicationManager.getApplication();  // Make sure we get the correct instance
                    if (rapp.getExecutionStatus().equalsIgnoreCase(RobotApplication.APP_STATUS_RUNNING)) {
                        toggle.setChecked(true);
                        Log.i(CLSS, String.format("updateStatusImage: set ball GREEN for %s", app.getApplicationName()));
                    }
                    else {
                        toggle.setChecked(false);
                        Log.i(CLSS, String.format("updateStatusImage: set ball YELLOW for %s", app.getApplicationName()));
                    }
                    toggle.setVisibility(View.VISIBLE);
                }
            }
        });
    }
    // =========================================== Checker Callbacks ====================================
    @Override
    public void handleConnectionError(String reason) {
        Log.w(CLSS, "handleConnectionError: " + reason);
        rosManager.setConnectionStatus(SBRosManager.CONNECTION_STATUS_UNCONNECTED);
        applicationManager.setApplication(null);
        if(getActivity()!=null) {
            SBWarningDialog warning = SBWarningDialog.newInstance("Error connecting to robot", reason);
            warning.show(getActivity().getFragmentManager(), DIALOG_TRANSACTION_KEY);
        }
    }

    /**
     * If this was a bluetooth error, then try wi-fi.
     * @param type network type (Bluetooth/Wi-fi)
     * @param reason error description
     */
    @Override
    public void handleNetworkError(String type,String reason) {
        Log.w(CLSS, String.format("handleNetworkError (%s): %s",type,reason));
        if(type.equalsIgnoreCase(SBConstants.NETWORK_BLUETOOTH)) {
            rosManager.setBluetoothError(reason);
            getActivity().runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    final Toast toast = Toast.makeText(getActivity().getBaseContext(), reason, Toast.LENGTH_LONG);
                    toast.show();
                }
            });
            Log.i(CLSS, "handleNetworkError: (bluetooth) " + reason);
            checkWifi();
        }
        else {
            rosManager.setWifiError(reason);
            SBWarningDialog warning = SBWarningDialog.newInstance("Network Error", reason);
            warning.show(getActivity().getFragmentManager(), DIALOG_TRANSACTION_KEY);
        }
    }

    // The application name is a global parameter of the robot. This method is called
    // once we've made contact with the robot. Instantiate the application object with its
    //
    // Display the full list of applications and mark this one as selected.
    @Override
    public void receiveApplication(String appName) {
        Log.w(CLSS, "receiveApplication: " + appName);
        List<RobotApplication> applicationList = applicationManager.getApplications();
        applicationManager.setApplication(appName);
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                RobotApplicationsAdapter adapter = (RobotApplicationsAdapter)getListAdapter();
                ListView listView = getListView();
                listView.setEnabled(true);
                int index = 0;
                for(RobotApplication app:applicationList) {
                    if(app.getApplicationName().equalsIgnoreCase(appName)) {
                        app.setExecutionStatus(RobotApplication.APP_STATUS_NOT_RUNNING);
                        listView.setItemChecked(index,true);
                        Log.i(CLSS, String.format("receiveApplication: selected application %s (%d)",appName,index));
                    }
                    index=index+1;
                }
                updateUI();
            }
        });
    }

    // Update robot connection status
    @Override
    public void receiveRobotConnection(RobotDescription robot) {
        Log.i(CLSS, "receiveRobotConnection: SUCCESS!");
        rosManager.setRobot(robot);
        rosManager.setConnectionStatus(SBRosManager.CONNECTION_STATUS_CONNECTED);
    }

    // The basic network connection is made. Now interrogate for robot characteristics.
    @Override
    public void receiveNetworkConnection() {
        Log.i(CLSS, "receiveNetworkConnection: SUCCESS!");
        MasterChecker checker = new MasterChecker(this);
        String master = dbManager.getSetting(SBConstants.ROS_MASTER_URI);
        checker.beginChecking(master);
        rosManager.setNetworkConnected(true);
    }


    //======================================== OnItemClickListener =====================================
    // If there has been no change to the list selection, do nothing. Otherwise,
    // disable the view to prevent clicks until we've heard from the robot.
    // Start the master checker to obtain robot characteristics - in particular the new application.
    // NOTE: We have circumvented the direct listener, via a click handler in the adapter class.
    //       Android has difficulty with the embedded button.
    @Override
    public void onItemClick(AdapterView<?> adapter, View v, int position,long rowId) {
        Log.i(CLSS, String.format("onItemClick: row %d",position));
        ListView listView = getListView();
        listView.setSelection(position);
        RobotApplication app = (RobotApplication)adapter.getItemAtPosition(position);
        if( applicationManager.getApplication()==null ||
               ! app.getApplicationName().equalsIgnoreCase(applicationManager.getApplication().getApplicationName() )) {

            applicationManager.setApplication(app.getApplicationName());
            command.execute(String.format(SET_APP_COMMAND,app.getApplicationName()));
            command.sudo(START_APP_COMMAND);
        }
    }
    //======================================== Button Callbacks ======================================
    // This dialog is passive and just dismisses itself.
    public void viewRobotClicked() {
        Log.i(CLSS, "View robot clicked");
        SBRobotViewDialog addDialog = new SBRobotViewDialog();
        addDialog.show(getActivity().getFragmentManager(), DIALOG_TRANSACTION_KEY);
    }

    /**
     * This is also called internally on a bluetooth connection error
     */
    public void connectRobotClicked() {
        Log.i(CLSS, "Connect robot clicked");
        RobotDescription robot = rosManager.getRobot();
        BluetoothManager bluetoothManager = (BluetoothManager)getActivity().getSystemService(BLUETOOTH_SERVICE);
        // If the robot is currently connected, we really mean "Disconnect"
        if( rosManager.getConnectionStatus().equals(SBRosManager.CONNECTION_STATUS_CONNECTED) ) {
            rosManager.setNetworkConnected(false);
            applicationManager.setApplication(null);
            if( bluetoothManager.getAdapter()!=null )  bluetoothManager.getAdapter().disable();
            WifiManager wifiManager = (WifiManager) getActivity().getApplicationContext().getSystemService(WIFI_SERVICE);
            wifiManager.disconnect();
            updateUI();

        }
        else if(!rosManager.hasBluetoothError()) {
            rosManager.setConnectionStatus(SBRosManager.CONNECTION_STATUS_CONNECTING);
            BluetoothChecker checker = new BluetoothChecker(this);
            String master = dbManager.getSetting(SBConstants.ROS_MASTER_URI);
            if( master.contains("xxx")) master = null;  // Our "hint" is not the real URI

            if (master != null) {
                RobotId id = new RobotId(master);
                // Supply device name that needs to match robot.
                String target = dbManager.getSetting(SBConstants.ROS_PAIRED_DEVICE);
                checker.beginChecking(id,bluetoothManager,target);
            }
            else {
                handleNetworkError(SBConstants.NETWORK_BLUETOOTH,"The MasterURI must be defined on the Settings panel");
            }
        }
        else {
            rosManager.setConnectionStatus(SBRosManager.CONNECTION_STATUS_CONNECTING);
            checkWifi();
        }
    }

    private void checkWifi() {
        WifiChecker checker = new WifiChecker(this);
        String master = dbManager.getSetting(SBConstants.ROS_MASTER_URI);
        if( master.contains("xxx")) master = null;  // Our "hint" is not the real URI

        if (master != null) {
            RobotId id = new RobotId(master);
            WifiManager wifiManager = (WifiManager) getActivity().getApplicationContext().getSystemService(WIFI_SERVICE);
            // Supply SSID in case robot is not the connected WiFi.
            id.setSSID(dbManager.getSetting(SBConstants.ROS_SSID));
            id.setWifiPassword(dbManager.getSetting(SBConstants.ROS_WIFIPWD));
            checker.beginChecking(id,wifiManager);
        }
        else {
            handleNetworkError(SBConstants.NETWORK_WIFI,"The MasterURI must be defined on the Settings panel");
        }
    }
    //  Start/stop toggle clicked:
    //    If this is not the application currently running on the robot,
    //    restart ROS on the robot with the newly desired application.
    //    Signal any interested listeners that the application has started
    public void startApplicationClicked() {
        if( applicationManager.getApplication()!=null ) {
            if( applicationManager.getApplication().getExecutionStatus().equals(RobotApplication.APP_STATUS_RUNNING) ) {
                Log.i(CLSS, "Stop application clicked");
                applicationManager.stopApplication();
            }
            else {
                Log.i(CLSS, "Start application clicked");
                applicationManager.startApplication(rosManager.getRobot());
            }
            Log.i(CLSS, String.format("startApplicationClicked: current position is %d",getListView().getSelectedItemPosition()));
            updateUI();
        }
    }
    //======================================== Update the UI ======================================
    /**
     * Keep the views in-sync with the model state
     */
    private void updateUI() {
        RobotDescription robot = rosManager.getRobot();
        Log.i(CLSS, String.format("updateUI robot:%s app:%s",(robot==null?"null":robot.getRobotName()),
                (applicationManager.getApplication()==null?"null":applicationManager.getApplication().getApplicationName())));

        Button button = (Button) contentView.findViewById(R.id.connectButton);
        button.setEnabled(true);
        if (!rosManager.getConnectionStatus().equalsIgnoreCase(SBRosManager.CONNECTION_STATUS_CONNECTED)) {
            button.setText(R.string.discoveryButtonConnect);
            rosManager.setRobot(null);
            robot = rosManager.getRobot();
        }
        else {
            button.setText((R.string.discoveryButtonDisconnect));
        }

        button = (Button) contentView.findViewById(R.id.viewButton);
        button.setEnabled(robot != null);

        ImageView iview = (ImageView) contentView.findViewById(R.id.robot_icon);
        if (robot == null) iview.setVisibility(View.INVISIBLE);
        else iview.setVisibility(View.VISIBLE);

        iview = (ImageView) contentView.findViewById(R.id.error_icon);
        if (robot == null) iview.setVisibility(View.INVISIBLE);
        else iview.setVisibility(View.VISIBLE);

        ProgressBar bar = (ProgressBar) contentView.findViewById(R.id.progress_circle);
        if (!rosManager.getConnectionStatus().equalsIgnoreCase(SBRosManager.CONNECTION_STATUS_CONNECTING))
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

        ListView listView = getListView();

        if (robot == null || applicationManager.getApplication()==null) {
            listView.setVisibility(View.INVISIBLE);
        }
        else {
            listView.setVisibility(View.VISIBLE);
            updateStatusOfCurrentApplication();
        }
    }

    private void updateStatusOfCurrentApplication() {
        int position = applicationManager.indexOfCurrentApplication();
        Log.i(CLSS, String.format("updateStatusForApplication %d",position));
        if( position<0 ) return;
        // Update the view
        View view = getListAdapter().getView(position,null,viewGroup);
        view.invalidate(0,0,view.getWidth(), view.getHeight());
    }
}
