/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.content.Context;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.text.Layout;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.ListAdapter;
import android.widget.ListView;
import android.widget.ProgressBar;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.db.SBDbManager;
import chuckcoughlin.sb.assistant.dialog.SBRobotViewDialog;
import chuckcoughlin.sb.assistant.dialog.SBWarningDialog;
import chuckcoughlin.sb.assistant.ros.SBRosApplicationManager;
import chuckcoughlin.sb.assistant.ros.SBRosManager;
import ros.android.appmanager.MasterChecker;
import ros.android.appmanager.SBRobotConnectionErrorListener;
import ros.android.appmanager.SBRobotConnectionHandler;
import ros.android.appmanager.WifiChecker;
import ros.android.util.RobotApplication;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

import static android.content.Context.WIFI_SERVICE;
import static chuckcoughlin.sb.assistant.common.SBConstants.DIALOG_TRANSACTION_KEY;

/**
 * Display the current robot. Robot attributes are read from the robot's own configuration parameters.
 * We can connect, select an application, then start or stop it.
 * Lifecycle methods are presented here in chronological order.
 */
public class DiscoveryFragment extends BasicAssistantListFragment implements SBRobotConnectionHandler {
    private final static String CLSS = "DiscoveryFragment";
    private SBDbManager dbManager;
    private SBRosManager rosManager;
    private SBRosApplicationManager applicationManager;
    private View contentView = null;


    // Called when the fragment's instance initializes
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(CLSS, "DiscoveryFragment.onCreate");
        super.onCreate(savedInstanceState);
        this.dbManager  = SBDbManager.getInstance();
        this.rosManager = SBRosManager.getInstance();
        this.applicationManager = SBRosApplicationManager.getInstance();
    }

    // Called to have the fragment instantiate its user interface view.
    // Inflate the view for the fragment based on layout XML. Populate
    // the text fields and application list from the database.
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        Log.i(CLSS, "DiscoveryFragment.onCreateView");
        this.contentView = inflater.inflate(R.layout.fragment_discovery, container, false);
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

        button = (Button) contentView.findViewById(R.id.startButton);
        ;
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startApplicationClicked();
            }
        });

        return contentView;
    }

    // Executes after onCreateView()
    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        Log.i(CLSS, "DiscoveryFragment.onViewCreated");
        updateUI();
    }

    // The host activity has been created.
    @Override
    public void onActivityCreated(Bundle savedInstanceState) {
        super.onActivityCreated(savedInstanceState);
        Log.i(CLSS, "DiscoveryFragment.onAxtivityCreated");

        // We populate the list whether or not there is a robot connection. We simply hide it as appropriate.
        List<RobotApplication> applicationList = SBRosApplicationManager.getInstance().getApplications();
        Log.i(CLSS, String.format("onActivityCreated: will display %d applications for all robots", applicationList.size()));
        RobotApplicationsAdapter adapter = new RobotApplicationsAdapter(getContext(), new ArrayList<>());
        setListAdapter(adapter);
        ListView listView = getListView();
        listView.setItemsCanFocus(true);
        listView.setVisibility(View.INVISIBLE);
        adapter.clear();
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
    // Use it to instantiate the application object with its
    // publish and subscribe definitions. Display the full list of applications
    // and mark this one as selected.
    @Override
    public void receiveApplication(String appName) {
        Log.w(CLSS, "receiveApplication: " + appName);
        List<RobotApplication> applicationList = SBRosApplicationManager.getInstance().getApplications();
        applicationManager.addListener((SBRobotConnectionErrorListener)this);
        applicationManager.setApplication(appName);
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                RobotApplicationsAdapter adapter = (RobotApplicationsAdapter)getListAdapter();
                adapter.clear();
                adapter.addAll(applicationList);
                int index = 0;
                for(RobotApplication app:applicationList) {
                    if(app.getApplicationName().equalsIgnoreCase(appName)) {
                        getListView().setSelection(index);
                        Log.i(CLSS, String.format("receiveApplication: selected application %s (%d)",appName,index));
                        break;
                    }
                    index=index+1;
                }
                updateUI();
                Log.i(CLSS, String.format("receiveApplication: updated UI for %d applications",applicationList.size()));
            }
        });
    }

    // Update robot connection status
    @Override
    public void receiveConnection(RobotDescription robot) {
        Log.i(CLSS, "receiveConnection: SUCCESS!");
        rosManager.updateRobot(robot);
        rosManager.setConnectionStatus(RobotDescription.CONNECTION_STATUS_CONNECTED);
    }

    // Once the wifi is connected, connect to the robot itself.
    // The "checker" wil fill in parameter values.
    @Override
    public void receiveWifiConnection() {
        Log.w(CLSS, "receiveWifiConnection: SUCCESS!");
        MasterChecker checker = new MasterChecker(this);
        String master = dbManager.getSetting(SBConstants.ROS_MASTER_URI);
        checker.beginChecking(master);
    }


    //======================================== Update the UI ======================================
    /**
     * Keep the views in-sync with the model state
     */
    private void updateUI() {
        RobotDescription robot = rosManager.getRobot();

        Button button = (Button) contentView.findViewById(R.id.connectButton);
        button.setEnabled(true);
        if (robot == null || !robot.getConnectionStatus().equalsIgnoreCase(RobotDescription.CONNECTION_STATUS_CONNECTED)) {
            button.setText(R.string.discoveryButtonConnect);
            rosManager.clearRobot();
            robot = rosManager.getRobot();
        }
        else {
            button.setText((R.string.discoveryButtonDisconnect));
        }

        button = (Button) contentView.findViewById(R.id.viewButton);
        button.setEnabled(robot != null);

        button = (Button) contentView.findViewById(R.id.startButton);
        button.setEnabled(robot != null && applicationManager.getApplication()!=null);
        if (robot == null || applicationManager.getApplication()==null ||
                !applicationManager.getApplication().getExecutionStatus().equals(RobotApplication.APP_STATUS_RUNNING))
            button.setText(R.string.discoveryButtonStart);
        else button.setText((R.string.discoveryButtonStop));

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

        ListView listView = getListView();
        if( listView.getAdapter()==null) return;  // When called from onActivityCreated
        Log.i(CLSS, String.format("DiscoveryFragment.updateUI robot:%s app:%s %d entries",(robot==null?"null":robot.getRobotName()),
                                  (applicationManager.getApplication()==null?"null":applicationManager.getApplication().getApplicationName()),
                                listView.getAdapter().getCount()));
        if (robot == null || applicationManager.getApplication()==null) {
            listView.setVisibility(View.INVISIBLE);
        }
        else {
            listView.setVisibility(View.VISIBLE);
        }
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

        @Override
        public View getView(int position, View convertView, ViewGroup parent) {
            Log.i(CLSS, String.format("RobotApplicationsAdapter.getView position =  %d", position));
            // Get the data item for this position
            RobotApplication app = getItem(position);

            // Check if an existing view is being reused, otherwise inflate the view
            if (convertView == null) {
                Log.i(CLSS, String.format("RobotApplicationsAdapter.getView convertView was null"));
                convertView = LayoutInflater.from(getContext()).inflate(R.layout.discovery_application_item, parent, false);
            }

            // Lookup view for data population
            TextView nameView = (TextView) convertView.findViewById(R.id.application_name);
            TextView descriptionView = (TextView) convertView.findViewById(R.id.application_description);
            TextView statusView = (TextView) convertView.findViewById(R.id.application_status);

            // Populate the data into the template view using the data object
            nameView.setText(app.getApplicationName());
            descriptionView.setText(app.getDescription());
            statusView.setText(app.getExecutionStatus());

            // Return the completed view to render on screen
            return convertView;
        }

        private void choose(int position) {

            Log.i(CLSS, String.format("RobotApplicationsAdapter.chose application %d",position));
            //SBRosApplicationManager.getInstance().setCurrentApplication(position);
        }
    }

    //======================================== Button Callbacks ======================================
    // This dialog is passive and just dismisses itself.
    public void viewRobotClicked() {
        Log.i(CLSS, "View robot clicked");
        SBRobotViewDialog addDialog = new SBRobotViewDialog();
        addDialog.show(getActivity().getFragmentManager(), DIALOG_TRANSACTION_KEY);
    }

    public void connectRobotClicked() {
        Log.i(CLSS, "Connect robot clicked");
        RobotDescription robot = rosManager.getRobot();
        WifiManager mgr = (WifiManager) getActivity().getApplicationContext().getSystemService(WIFI_SERVICE);
        // If the robot is currently connected, we really mean "Disconnect"
        if( rosManager.getConnectionStatus().equals(RobotDescription.CONNECTION_STATUS_CONNECTED) ) {
            mgr.disconnect();
            rosManager.setConnectionStatus(RobotDescription.CONNECTION_STATUS_UNCONNECTED);
            rosManager.clearRobot();
            applicationManager.shutdown();
        }
        else {
            WifiChecker checker = new WifiChecker(this);
            String master = dbManager.getSetting(SBConstants.ROS_MASTER_URI);
            if( master.contains("xxx")) master = null;  // Our "hint" is not the real URI

            if (master != null) {
                RobotId id = new RobotId(master);
                // Supply SSID in case robot is not the connected WiFi.
                id.setSSID(dbManager.getSetting(SBConstants.ROS_SSID));
                id.setWifiPassword(dbManager.getSetting(SBConstants.ROS_WIFIPWD));
                checker.beginChecking(id,mgr);
            }
            else {
                handleNetworkError("The MasterURI must be defined on the Settings panel");
            }
        }
    }

    // On start, create all the publish/subscribe settings for the application.
    public void startApplicationClicked() {
        Log.i(CLSS, "Start/stop application clicked");
        if( applicationManager.getApplication()!=null ) {
            if( applicationManager.getApplication().getExecutionStatus().equals(RobotApplication.APP_STATUS_RUNNING) ) {
                applicationManager.stopApplication();
            }
            else {
                applicationManager.startApplication();
            }
        }
    }

}
