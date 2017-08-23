/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */
package chuckcoughlin.sbcontrol.tab.logs;

import android.content.Context;
import android.support.v7.widget.RecyclerView;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import chuckcoughlin.sbcontrol.R;


/**
 * This a link between a RecyclerView and the data backstop.
 */

public class RecyclerAdapter extends RecyclerView.Adapter<LogViewHolder> {
    private static final String TAG = RecyclerAdapter.class.getSimpleName();
    private int nItems;

    /**
     * Adapter between the recycler and data source for log messages
     * @param count the initial number of items
     */
    public RecyclerAdapter(int count) {
        this.nItems = count;
    }

    @Override
    public LogViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
        Context context = parent.getContext();
        int layoutIdForListItem = R.id.logs_recyclable_item;
        LayoutInflater inflater = LayoutInflater.from(context);
        boolean shouldAttachToParentImmediately = false;

        View view = inflater.inflate(layoutIdForListItem,parent,shouldAttachToParentImmediately);
        LogViewHolder holder = new LogViewHolder(view);
        return holder;
    }

    /**
     * This methgod is called by the RecyclerView to display the data at a particluar
     * position. We update the contents of the ViewHolder to display the correct
     * information for this position in the list.
     * @param holder the ViewHolder which should be updated
     * @param position the index of the item within the adapter's data set.
     */
    @Override
    public void onBindViewHolder(LogViewHolder holder, int position) {
        Log.i(TAG,"# "+position);
        holder.bind(position);
    }

    @Override
    public int getItemCount() {
        return nItems;
    }
}