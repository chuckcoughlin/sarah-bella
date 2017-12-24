/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.app.Dialog;
import android.app.FragmentTransaction;
import android.content.Context;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.app.FragmentManager;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ListAdapter;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import java.util.List;
import java.util.Map;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.dialog.SBBasicDialogFragment;
import chuckcoughlin.sb.assistant.dialog.SBDialogCallbackHandler;
import chuckcoughlin.sb.assistant.dialog.SBRobotAddDialog;
import chuckcoughlin.sb.assistant.dialog.SBRobotScanDialog;
import chuckcoughlin.sb.assistant.ros.SBRosHelper;
import ros.android.util.InvalidRobotDescriptionException;
import ros.android.util.RobotDescription;
import ros.android.util.RobotId;

import static chuckcoughlin.sb.assistant.common.SBConstants.DIALOG_TRANSACTION_KEY;

/**
 * Search the networks for robots. Based on ros.activity.MasterChooserActivity
 * Lifecycle methods are presented in chronological order.
 */
public class DiscoveryFragment extends BasicAssistantListFragment implements SBDialogCallbackHandler {
    private final static String CLSS = "DiscoveryFragment";
    private SBRosHelper rosHelper;

    // Called when the fragment's instance initializes
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.rosHelper = SBRosHelper.getInstance();
    }

    // Called to have the fragment instantiate its user interface view.
    // Inflate the view for the fragment based on layout XML. Populate
    // the text fields from the database.
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View contentView = inflater.inflate(R.layout.fragment_discovery, container, false);
        TextView textView = contentView.findViewById(R.id.fragmentDiscoveryText);
        textView.setText(R.string.fragmentDiscoveryLabel);

        Button button = (Button) contentView.findViewById(R.id.addButton);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                addRobotClicked();
            }
        });
        button = (Button) contentView.findViewById(R.id.clearButton);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                clearRobotClicked();
            }
        });
        button = (Button) contentView.findViewById(R.id.scanButton);
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                scanRobotClicked();
            }
        });
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
        List<RobotDescription> robotList = rosHelper.getRobots();
        Log.i(CLSS,String.format("onActivityCreated: will display %d robot descriptions",robotList.size()));
        RobotListAdapter adapter = new RobotListAdapter(getContext(),robotList);
        setListAdapter(adapter);
        getListView().setItemsCanFocus(true);
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
        if( dfrag.getDialogType().equalsIgnoreCase(SBRobotAddDialog.CLSS)) {
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
                    rosHelper.addRobot(robot);
                    RobotListAdapter adapter = (RobotListAdapter) getListAdapter();
                    adapter.add(robot);
                    adapter.notifyDataSetInvalidated();
                    Log.i(CLSS,String.format("handleDialogResults added %s - now have %d",robot.getRobotName(),rosHelper.getRobots().size()));
                }
            }
        }
    }


    //======================================== Array Adapter ======================================
    public class RobotListAdapter extends ArrayAdapter<RobotDescription> implements ListAdapter {

        public RobotListAdapter(Context context, List<RobotDescription> values) {
            super(context,R.layout.robot_item, values);
        }

        @Override
        public long getItemId(int position) {
            return getItem(position).hashCode();
        }

        @Override
        public View getView(int position, View convertView, ViewGroup parent) {
            Log.i(CLSS,String.format("RobotListAdapter.getView position =  %d",position));
            // Get the data item for this position
            RobotDescription description = getItem(position);
            // Check if an existing view is being reused, otherwise inflate the view
            if (convertView == null) {
                Log.i(CLSS,String.format("RobotListAdapter.getView convertView was null"));
                convertView = LayoutInflater.from(getContext()).inflate(R.layout.robot_item, parent, false);
            }

            // Lookup view for data population
            TextView nameView = (TextView) convertView.findViewById(R.id.name);
            TextView uriView = (TextView)  convertView.findViewById(R.id.uri);
            TextView statusView=(TextView) convertView.findViewById(R.id.status);

            // Populate the data into the template view using the data object
            nameView.setText(description.getRobotName());
            uriView.setText(description.getRobotId().getMasterUri());
            statusView.setText(description.getConnectionStatus());
            /*
            listView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
                @Override
                public void onItemClick(AdapterView<?> parent, View v, int position, long id) {
                    choose(position);
                }
            });

            Log.i(CLSS,String.format("RobotListAdapter.getView set %s(%s) = %s",description.getRobotName(),description.getRobotId(),description.getConnectionStatus()));
            int index = 0;
            int count = getCount();
            while( index<count ) {
                RobotDescription robot = getItem(index);
                if( robot != null && robot.equals( rosHelper.getCurrentRobot() )) {
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
            SBRosHelper.getInstance().setCurrentRobot(position);
        }
    }
    //======================================== Button Callbacks ======================================
    public void addRobotClicked() {
        Log.i(CLSS,"Add robot clicked");
        SBRobotAddDialog addDialog = new SBRobotAddDialog();
        addDialog.setHandler(this);
        addDialog.show(getActivity().getFragmentManager(), DIALOG_TRANSACTION_KEY);
    }
    public void clearRobotClicked() {
        Log.i(CLSS,"Clear robots clicked");
        rosHelper.clearRobots();
        RobotListAdapter adapter = (RobotListAdapter)getListAdapter();
        adapter.clear();
        adapter.notifyDataSetInvalidated();
    }
    public void scanRobotClicked() {
        Log.i(CLSS,"Scan for robots clicked");
        SBRobotScanDialog scanDialog = new SBRobotScanDialog();
        scanDialog.setHandler(this);
        scanDialog.show(getActivity().getFragmentManager(), DIALOG_TRANSACTION_KEY);
    }


}
