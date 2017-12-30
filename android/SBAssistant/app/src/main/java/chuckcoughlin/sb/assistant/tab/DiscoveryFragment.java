/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.content.Context;
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

import java.util.List;

import app_manager.App;
import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.dialog.SBBasicDialogFragment;
import chuckcoughlin.sb.assistant.dialog.SBDialogCallbackHandler;
import chuckcoughlin.sb.assistant.dialog.SBRobotCreateDialog;
import chuckcoughlin.sb.assistant.ros.SBRosApplicationManager;
import chuckcoughlin.sb.assistant.ros.SBRosManager;
import ros.android.util.RobotDescription;

import static chuckcoughlin.sb.assistant.common.SBConstants.DIALOG_TRANSACTION_KEY;

/**
 * Display the current robot. Provide for its creation if it doesn't exist and for
 * its editing if it does. We can validate, if needed and then start or stop applications.
 * Lifecycle methods are presented here in chronological order.
 */
public class DiscoveryFragment extends BasicAssistantListFragment implements SBDialogCallbackHandler {
    private final static String CLSS = "DiscoveryFragment";
    private SBRosManager rosManager;
    private SBRosApplicationManager applicationManager;
    private View contentView = null;


    // Called when the fragment's instance initializes
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(CLSS,"DiscoveryFragment.onCreate");
        super.onCreate(savedInstanceState);
        this.rosManager = SBRosManager.getInstance();
        this.applicationManager = SBRosApplicationManager.getInstance();
    }

    // Called to have the fragment instantiate its user interface view.
    // Inflate the view for the fragment based on layout XML. Populate
    // the text fields from the database.
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        Log.i(CLSS,"DiscoveryFragment.onCreateView");
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

        button = (Button) contentView.findViewById(R.id.clearButton);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                clearRobotClicked();
            }
        });
        button = (Button) contentView.findViewById(R.id.validateButton);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                validateRobotClicked();
            }
        });
        button = (Button) contentView.findViewById(R.id.startButton);;
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                validateRobotClicked();
            }
        });
        updateUI();
        return contentView;
    }

    // Executes after onCreateView()
    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
    }

    // The host activity has been created.
    @Override
    public void onActivityCreated(Bundle savedInstanceState) {
        super.onActivityCreated(savedInstanceState);
        RobotDescription robot = rosManager.getRobot();
        if( robot!=null ) {
            List<App> applicationList = SBRosApplicationManager.getInstance().getApplications();
            Log.i(CLSS, String.format("onActivityCreated: will display %d applications for %s", applicationList.size(), rosManager.getRobot().getRobotName()));
            RobotApplicationsAdapter adapter = new RobotApplicationsAdapter(getContext(), applicationList);
            setListAdapter(adapter);
            getListView().setItemsCanFocus(true);
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

    // =========================================== Dialog Callback =====================================

    public void handleDialogResult(SBBasicDialogFragment dfrag) {
        Log.i(CLSS,String.format("handleDialogResults for %s",dfrag.getDialogType()));
        if( dfrag.getDialogType().equalsIgnoreCase(SBRobotCreateDialog.CLSS)) {
            String btn = dfrag.getSelectedButton();
            Log.i(CLSS,String.format("handleDialogResults clicked on %s",btn));
            if( btn.equalsIgnoreCase(SBConstants.DIALOG_BUTTON_ADD)) {
                String msg = dfrag.getErrorMessage();
                Log.i(CLSS,String.format("handleDialogResults error is %s",msg));
                RobotDescription robot = (RobotDescription) dfrag.getPayload();

                if (msg != null && !msg.isEmpty()) {
                    final Toast toast = Toast.makeText(getActivity(), msg, Toast.LENGTH_LONG);
                    toast.show();
                }
                else if (robot != null) {
                    if( rosManager.getRobot()==null) {
                        rosManager.createRobot(robot);
                    }
                    else{
                        rosManager.updateRobot(robot);
                    }

                    RobotApplicationsAdapter adapter = (RobotApplicationsAdapter) getListAdapter();
                    List<App> apps = applicationManager.getApplications();
                    adapter.clear();
                    for( App app:apps) {
                        adapter.add(app);
                        Log.i(CLSS,String.format("handleDialogResults added %s - now have %d",app.getName(), applicationManager.getApplicationCount()));
                    }
                    adapter.notifyDataSetInvalidated();

                }
                updateUI();
            }
        }
    }


    //======================================== Array Adapter ======================================
    public class RobotApplicationsAdapter extends ArrayAdapter<App> implements ListAdapter {

        public RobotApplicationsAdapter(Context context, List<App> values) {
            super(context,R.layout.ros_application_item, values);
        }

        @Override
        public long getItemId(int position) {
            return getItem(position).hashCode();
        }

        @Override
        public View getView(int position, View convertView, ViewGroup parent) {
            Log.i(CLSS,String.format("RobotApplicationsAdapter.getView position =  %d",position));
            // Get the data item for this position
            App app = getItem(position);
            // Check if an existing view is being reused, otherwise inflate the view
            if (convertView == null) {
                Log.i(CLSS,String.format("RobotApplicationsAdapter.getView convertView was null"));
                convertView = LayoutInflater.from(getContext()).inflate(R.layout.ros_application_item, parent, false);
            }

            // Lookup view for data population
            TextView nameView = (TextView) convertView.findViewById(R.id.name);
            TextView uriView = (TextView)  convertView.findViewById(R.id.uri);
            TextView statusView=(TextView) convertView.findViewById(R.id.status);

            // Populate the data into the template view using the data object
            nameView.setText(app.getName());
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
        if( robot==null ) button.setText(R.string.discoveryButtonDefine);
        else              button.setText((R.string.discoveryButtonEdit));

        button = (Button) contentView.findViewById(R.id.clearButton);
        button.setEnabled(robot!=null);

        button = (Button) contentView.findViewById(R.id.validateButton);
        button.setEnabled(robot!=null);

        button = (Button) contentView.findViewById(R.id.startButton);
        button.setEnabled(robot!=null);

        ImageView iview = (ImageView) contentView.findViewById(R.id.robot_icon);
        if(robot==null) iview.setVisibility(View.INVISIBLE);
        else            iview.setVisibility(View.VISIBLE);

        iview = (ImageView) contentView.findViewById(R.id.error_icon);
        if(robot==null) iview.setVisibility(View.INVISIBLE);
        else            iview.setVisibility(View.VISIBLE);

        ProgressBar bar = (ProgressBar) contentView.findViewById(R.id.progress_circle);
        if(robot==null && !robot.getConnectionStatus().equalsIgnoreCase(RobotDescription.CONNECTING) ) bar.setVisibility(View.INVISIBLE);
        else   {
            bar.setVisibility(View.VISIBLE);
            bar.setIndeterminate(true);
        }
        TextView tview = (TextView) contentView.findViewById(R.id.robot_name);
        if(robot==null) tview.setVisibility(View.INVISIBLE);
        else {
            tview.setText(robot.getRobotName());
            tview.setVisibility(View.VISIBLE);
        }

        tview = (TextView) contentView.findViewById(R.id.master_uri);
        if(robot==null) tview.setVisibility(View.INVISIBLE);
        else {
            tview.setText(robot.getRobotId().getMasterUri());
            tview.setVisibility(View.VISIBLE);
        }

        tview = (TextView) contentView.findViewById(R.id.status);
        if(robot==null) tview.setVisibility(View.INVISIBLE);
        else {
            tview.setText(applicationManager.getStatusAsString());
            tview.setVisibility(View.VISIBLE);
        }
    }
    //======================================== Button Callbacks ======================================
    public void defineRobotClicked() {
        Log.i(CLSS,"Add robot clicked");
        SBRobotCreateDialog addDialog = new SBRobotCreateDialog();
        addDialog.setHandler(this);
        addDialog.show(getActivity().getFragmentManager(), DIALOG_TRANSACTION_KEY);
    }
    public void clearRobotClicked() {
        Log.i(CLSS,"Clear robots clicked");
        applicationManager.clearApplications();
        RobotApplicationsAdapter adapter = (RobotApplicationsAdapter)getListAdapter();
        adapter.clear();
        adapter.notifyDataSetInvalidated();
    }
    public void validateRobotClicked() {
        Log.i(CLSS,"Validate robot clicked");
        /*
        SBRobotScanDialog scanDialog = new SBRobotScanDialog();
        scanDialog.setHandler(this);
        scanDialog.show(getActivity().getFragmentManager(), DIALOG_TRANSACTION_KEY);
        */
    }


}
