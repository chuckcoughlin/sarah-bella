/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.app.Activity;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.BaseAdapter;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.db.SBDbHelper;
import chuckcoughlin.sb.assistant.utilities.NameValue;
import chuckcoughlin.sb.assistant.utilities.SBConstants;

/**
 * Display the current values of global application settings and allow
 * editing.
 */

public class SettingsFragment extends BasicAssistantFragment {
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
        ListView listView = (ListView)contentView.findViewById(R.id.settingsListView);
        List<NameValue> nvpairs = getNameValues();
        NameValue [] nvarray = nvpairs.toArray(new NameValue[nvpairs.size()]);
        adapter = new SettingsListAdapter(container.getContext(),nvarray);
        listView.setAdapter(adapter);
        return contentView;
    }

    // Called when the view previously created by onCreateView has been detached from the fragment.
    // Save the view values to the database.
    @Override
    public void onDestroyView() {
        saveViewValues();
        super.onDestroyView();
    }

    // Read name/value pairs from the database.
    private List<NameValue> getNameValues() {
        List<NameValue> list = new ArrayList<>();
        SQLiteDatabase database = new SBDbHelper(getContext()).getReadableDatabase();
        String[] args = new String[0];   // Use for PreparedStatement
        String SQL = "SELECT name,value FROM Settings ORDER BY Name";
        Cursor cursor = database.rawQuery(SQL,args);
        cursor.moveToFirst();
        NameValue nv = new NameValue();
        while( !cursor.isAfterLast() ) {
            nv.setName(cursor.getString(0));
            nv.setValue(cursor.getString(1));
            list.add(nv);
            cursor.moveToNext();
        }
        cursor.close();
        database.close();
        return list;
    }

    /** Save view values to the database
     * @param view the parent view
     */
    private void saveViewValues() {
        if( adapter==null) return;

        SQLiteDatabase database = new SBDbHelper(getContext()).getWritableDatabase();
        String SQL = "UPDATE Settings set value = ? WHERE name = ?";
        String[] bindArgs = new String[2];
        int count = adapter.getCount();
        int index = 0;
        while( index<count) {
            NameValue nv = adapter.getItem(index);
            bindArgs[0] = nv.getValue();
            bindArgs[1] = nv.getName();
            database.execSQL(SQL,bindArgs);
            index++;
        }
        database.close();
    }


    private class SettingsListAdapter extends ArrayAdapter<NameValue> {

        public SettingsListAdapter(Context context, NameValue[] values) {
            super(context, 0, values);
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

            // Check if an existing view is being reused, else inflate the view
            if (convertView == null) {
                convertView = getLayoutInflater().inflate(R.layout.fragment_settings, parent, false);
            }

            // Get the data item for this position
            NameValue nv = getItem(position);
            // Lookup view for data population
            TextView nameView  = (TextView) convertView.findViewById(R.id.settingsNameView);
            if( nameView==null) return convertView;
            Log.i(CLSS,"=====================");
            //Log.i(CLSS,String.format("getView %d : %s ",position,(nv==null?"NULL":nv.getName())));
            EditText valueView = (EditText) convertView.findViewById(R.id.settingsEditView);
            if( valueView==null) return convertView;
            Log.i(CLSS,"=====================");
            // Populate the data into the template view using the data object
            nameView.setText(nv.getName());
            valueView.setText(nv.getValue());
            // Return the completed view to render on screen
            return convertView;


        }

    }

}
