/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.annotation.SuppressLint;
import android.content.Context;
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
import android.widget.Toast;

import java.util.List;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.db.SBDbHelper;
import chuckcoughlin.sb.assistant.utilities.NameValue;

/**
 * Display the current values of global application settings and allow
 * editing.
 */
public class SettingsFragment extends BasicAssistantListFragment implements AdapterView.OnItemClickListener {
    private final static String CLSS = "SettingsFragment";

    public SettingsFragment() {
        super();
    }

    @Override
    public void onActivityCreated(Bundle savedInstanceState) {
        super.onActivityCreated(savedInstanceState);
        List<NameValue> nvpairs = SBDbHelper.getInstance().getSettings();
        NameValue [] nvarray = nvpairs.toArray(new NameValue[nvpairs.size()]);
        Log.i(CLSS,String.format("onActivityCreated: will display %d name-values",nvarray.length));
        SettingsListAdapter adapter = new SettingsListAdapter(getContext(),nvarray);
        setListAdapter(adapter);
        getListView().setOnItemClickListener(this);
    }

    // Called to have the fragment instantiate its user interface view.
    // Inflate the view for the fragment based on layout XML. Populate
    // the text fields from the database.
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View contentView = inflater.inflate(R.layout.fragment_settings, container, false);
        TextView textView = contentView.findViewById(R.id.fragmentSettingsText);
        textView.setText(R.string.fragmentSettingsLabel);
        return contentView;
    }

    public class SettingsListAdapter extends ArrayAdapter<NameValue> implements ListAdapter {

        public SettingsListAdapter(Context context, NameValue[] values) {
            super(context,R.layout.settings_item, values);
        }

        /**
         * We've verified that this works.
         */
        @Override
        public int getCount() {
            //Log.i(CLSS,String.format("SettingsListAdapter.getCount =  %d",super.getCount()));
            return super.getCount();
        }

        @Override
        public long getItemId(int position) {
            return getItem(position).hashCode();
        }

        @Override
        public View getView(int position, View convertView, ViewGroup parent) {
            Log.i(CLSS,String.format("SettingsListAdapter.getView position =  %d",position));
            // Get the data item for this position
            NameValue nv = getItem(position);
            // Check if an existing view is being reused, otherwise inflate the view
            if (convertView == null) {
                Log.i(CLSS,String.format("SettingsListAdapter.getView convertView was null"));
                convertView = LayoutInflater.from(getContext()).inflate(R.layout.settings_item, parent, false);
            }
            // Lookup view for data population
            TextView nameView = (TextView) convertView.findViewById(R.id.settingsNameView);
            EditText editText = (EditText) convertView.findViewById(R.id.settingsEditView);
            // Populate the data into the template view using the data object
            nameView.setText(nv.getName());
            editText.setText(nv.getValue());
            Log.i(CLSS,String.format("SettingsListAdapter.getView set %s = %s",nv.getName(),nv.getValue()));
            // Return the completed view to render on screen
            return convertView;
        }
    }

    // ============================= itemClickListener =============================
    @Override
    public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
        Toast.makeText(getContext(), "Item: " + position, Toast.LENGTH_SHORT).show();
    }
}
