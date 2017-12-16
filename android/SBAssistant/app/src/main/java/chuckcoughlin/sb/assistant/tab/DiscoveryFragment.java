/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.app.Activity;
import android.content.ContentValues;
import android.content.Context;
import android.content.Intent;
import android.database.Cursor;
import android.net.Uri;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.ListAdapter;
import android.widget.ListView;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.db.SBDbHelper;
import chuckcoughlin.sb.assistant.ros.SBRosHelper;
import chuckcoughlin.sb.assistant.utilities.NameValue;
import ros.android.util.RobotDescription;
import ros.android.util.RobotsContentProvider;

/**
 * Search the networks for robots. Based on ros.activity.MasterChooserActivity
 * Lifecycle methods are presented in chronological order.
 */
public class DiscoveryFragment extends BasicAssistantListFragment  {
    private final static String CLSS = "DiscoveryFragment";
    private boolean[] selections;
    private final SBRosHelper rosHelper;

    public DiscoveryFragment() {
        super();
        this.rosHelper = SBRosHelper.getInstance();
    }

    // Called when the fragment's instance initializes
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setTitle(getString(R.string.fragmentDiscoverySelectMaster));
        rosHelper.readRobotList();
    }

    // Called to have the fragment instantiate its user interface view.
    // Inflate the view for the fragment based on layout XML. Populate
    // the text fields from the database.
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View contentView = inflater.inflate(R.layout.fragment_discovery, container, false);
        TextView textView = contentView.findViewById(R.id.fragmentDiscoveryText);
        textView.setText(R.string.fragmentDiscoveryLabel);
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

    // =========================================== Private helper methods =====================================

    private void choose(int position) {
        SBRosHelper.getInstance().setCurrentRobot(position);
    }


    //======================================== Array Adapter ======================================
    public class RobotListAdapter extends ArrayAdapter<RobotDescription> implements ListAdapter {
        private final List<RobotDescription> robotList;
        public RobotListAdapter(Context context, List<RobotDescription> values) {
            super(context,R.layout.robot_item, values);
            this.robotList = values;
        }

        @Override
        public long getItemId(int position) {
            return getItem(position).hashCode();
        }

        @Override
        public View getView(int position, View convertView, ViewGroup parent) {
            // Log.i(CLSS,String.format("SettingsListAdapter.getView position =  %d",position));
            // Get the data item for this position
            RobotDescription description = getItem(position);
            // Check if an existing view is being reused, otherwise inflate the view
            if (convertView == null) {
                // Log.i(CLSS,String.format("SettingsListAdapter.getView convertView was null"));
                convertView = LayoutInflater.from(getContext()).inflate(R.layout.robot_item, parent, false);
            }

            ListView listView = (ListView)convertView;
            // Lookup view for data population
            TextView nameView = (TextView) convertView.findViewById(R.id.name);
            TextView uriView = (EditText) convertView.findViewById(R.id.uri);
            TextView statusView = (EditText) convertView.findViewById(R.id.uri);

            // Populate the data into the template view using the data object
            nameView.setText(description.getRobotName());
            uriView.setText(description.getRobotId().getMasterUri());
            statusView.setText(description.getConnectionStatus());
            listView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
                @Override
                public void onItemClick(AdapterView<?> parent, View v, int position, long id) {
                    choose(position);
                }
            });

            Log.i(CLSS,String.format("RobotListAdapter.getView set %s(%s) = %s",description.getRobotName(),description.getRobotId(),description.getConnectionStatus()));
            int index = 0;
            for( RobotDescription robot: robotList ) {
                if( robot != null && robot.equals( rosHelper.getCurrentRobot() )) {
                    Log.i(CLSS, "Highlighting index " + index);
                    listView.setItemChecked(index, true);
                    break;
                }
                index++;
            }
            // Return the completed view to render on screen
            return convertView;
        }
    }

}
