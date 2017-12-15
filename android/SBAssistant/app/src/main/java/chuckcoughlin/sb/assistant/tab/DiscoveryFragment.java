/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.app.Activity;
import android.content.Context;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.ListAdapter;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.db.SBDbHelper;
import chuckcoughlin.sb.assistant.utilities.NameValue;
import ros.android.util.MasterChooser;
import ros.android.util.RobotDescription;

/**
 * Search the networks for robots. Based on ros.activity.MasterChooserActivity
 * Lifecycle methods are presented in chronological order.
 */
public class DiscoveryFragment extends BasicAssistantListFragment  {
    private final static String CLSS = "DiscoveryFragment";
    private List<RobotDescription> robots;
    private MasterChooser currentRobotAccessor;
    private boolean[] selections;

    public DiscoveryFragment() {
        super();
        robots = new ArrayList<>();
        currentRobotAccessor = new MasterChooser(this.getActivity());
    }


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
// add your code here which executes when fragment's instance initializes
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

    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
// add your code here which executes after the execution of onCreateView() method.

    }
    // The host activity has been created.
    @Override
    public void onActivityCreated(Bundle savedInstanceState) {
        super.onActivityCreated(savedInstanceState);
        List<NameValue> nvpairs = SBDbHelper.getInstance().getSettings();
        NameValue [] nvarray = nvpairs.toArray(new NameValue[nvpairs.size()]);
        Log.i(CLSS,String.format("onActivityCreated: will display %d name-values",nvarray.length));
        SettingsListAdapter adapter = new SettingsListAdapter(getContext(),nvarray);
        setListAdapter(adapter);
        getListView().setItemsCanFocus(true);
    }

    @Override
    public void onStart() {
        super.onStart();
// add your code here which executes when the Fragment gets visible.
    }

    @Override
    public void onResume() {
        super.onResume();
// add your code here which executes when the Fragment is visible and intractable.
    }

    @Override
    public void onPause() {
        super.onPause();
// add your code here which executes when user leaving the current fragment or fragment is no longer intractable.
    }
    @Override
    public void onStop() {
        super.onStop();
// add your code here which executes Fragment going to be stopped.
    }
    @Override
    public void onDestroyView() {
        super.onDestroyView();
// add your code here which executes when the view's and other related resources created in onCreateView() method are removed
    }
    @Override
    public void onDestroy() {
        super.onDestroy();
// add your code here which executes when the final clean up for the Fragment's state is needed.
    }

    @Override
    public void onDetach() {
        super.onDetach();
// add your code here which executes when fragment has been disassociated from its hosting activity
    }

    public class SettingsListAdapter extends ArrayAdapter<NameValue> implements ListAdapter {

        public SettingsListAdapter(Context context, NameValue[] values) {
            super(context,R.layout.settings_item, values);
        }

        @Override
        public long getItemId(int position) {
            return getItem(position).hashCode();
        }

        @Override
        public View getView(int position, View convertView, ViewGroup parent) {
            // Log.i(CLSS,String.format("SettingsListAdapter.getView position =  %d",position));
            // Get the data item for this position
            NameValue nv = getItem(position);
            // Check if an existing view is being reused, otherwise inflate the view
            if (convertView == null) {
                // Log.i(CLSS,String.format("SettingsListAdapter.getView convertView was null"));
                convertView = LayoutInflater.from(getContext()).inflate(R.layout.settings_item, parent, false);
            }
            // Lookup view for data population
            TextView nameView = (TextView) convertView.findViewById(R.id.settingsNameView);
            EditText editText = (EditText) convertView.findViewById(R.id.settingsEditView);
            // Populate the data into the template view using the data object
            nameView.setText(nv.getName());
            editText.setText(nv.getValue());
            editText.setOnFocusChangeListener(new View.OnFocusChangeListener() {
                @Override
                public void onFocusChange(View v, boolean hasFocus) {
                    /*
                     * When focus is lost save the entered value both into the current array
                     * and the database
                     */
                    if (!hasFocus) {
                        Log.i(CLSS,String.format("SettingsListAdapter.getView.onFocusChange %d = %s",position,((EditText) v).getText().toString()));
                        nv.setValue(((EditText)v).getText().toString());
                        SBDbHelper.getInstance().updateSetting(nv);
                    }
                }
            });


            Log.i(CLSS,String.format("SettingsListAdapter.getView set %s = %s",nv.getName(),nv.getValue()));
            // Return the completed view to render on screen
            return convertView;
        }
    }

}
