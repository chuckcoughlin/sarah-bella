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
import android.widget.EditText;
import android.widget.ListAdapter;
import android.widget.ListView;
import android.widget.TextView;

import java.util.List;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.db.SBDbHelper;
import chuckcoughlin.sb.assistant.utilities.NameValue;

/**
 * Display the current values of global application settings and allow
 * editing.
 */

public class SettingsFragment extends BasicAssistantListFragment {
    private final static String CLSS = "SettingsFragment";
    private SettingsListAdapter adapter = null;
    private View contentView = null;

    public SettingsFragment() {
        super();

    }

    // Called to have the fragment instantiate its user interface view.
    // Inflate the view for the fragment based on layout XML. Populate
    // the text fields from the database.
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {

        contentView = inflater.inflate(R.layout.fragment_settings, container, false);
        TextView textView = contentView.findViewById(R.id.fragmentSettingsText);
        textView.setText(R.string.fragmentSettingsLabel);

        List<NameValue> nvpairs = SBDbHelper.getInstance().getSettings();
        NameValue [] nvarray = nvpairs.toArray(new NameValue[nvpairs.size()]);
        Log.i(CLSS,String.format("onCreateView: displaying %d name-values",nvarray.length));
        adapter = new SettingsListAdapter(container.getContext(),nvarray);
        setListAdapter(adapter);
        return contentView;
    }

    private class SettingsListAdapter extends ArrayAdapter<NameValue> implements ListAdapter {

        public SettingsListAdapter(Context context, NameValue[] values) {
            super(context, 0, values);
        }

        /**
         * We've verified that this works.
         */
        @Override
        public int getCount() {
            Log.i(CLSS,String.format("SettingsListAdapter.getCount =  %d",super.getCount()));
            return super.getCount();
        }

        @Override
        public NameValue getItem(int position) {
            Log.i(CLSS,String.format("SettingsListAdapter.getItem at %d = %s",position,super.getItem(position).getName()));
            return super.getItem(position);
        }

        @Override
        public long getItemId(int position) {
            NameValue nv = getItem(position);
            return 10000*nv.getName().hashCode()+nv.getValue().hashCode();
        }
        /**
         * Extract the nth view in the list. In our case this is a RelativeLayout
         * containing a TextView (name) and EditView (value).
         *
         * @param position
         * @param convertView convert this view for display
         * @param parent
         * @return
         */
        @Override
        public View getView(int position, View convertView, ViewGroup parent) {
            Log.i(CLSS,String.format("SettingsListAdapter.getView position %d",position));
            // Check if an existing view is being reused, else inflate the view
            if (convertView == null) {
                //Log.i(CLSS,String.format("SettingsListAdapter.getView convert view was null"));
                convertView = getLayoutInflater().inflate(R.layout.settings_item, parent, false);
            }
            // Get the data item for this position
            NameValue nv = getItem(position);
            Log.i(CLSS,String.format("SettingsListAdapter.getView name at %d = %s",position,nv.getName()));
            // Lookup view for data population
            TextView nameView  = (TextView) convertView.findViewById(R.id.settingsNameView);
            if( nameView==null) return convertView;
            //Log.i(CLSS,String.format("getView %d : %s ",position,(nv==null?"NULL":nv.getName())));
            EditText valueView = (EditText) convertView.findViewById(R.id.settingsEditView);
            if( valueView==null) return convertView;
            Log.i(CLSS,String.format("===================== %s",nv.getValue()));
            // Populate the data into the template view using the data object
            nameView.setText(nv.getName());
            valueView.setText(nv.getValue());
            // Update the database
            SBDbHelper.getInstance().updateSetting(nv);
            // Return the completed view to render on screen
            return convertView;
        }

    }

}
