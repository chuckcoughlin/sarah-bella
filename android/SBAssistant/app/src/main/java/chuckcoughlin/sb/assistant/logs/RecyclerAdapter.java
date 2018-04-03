/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.logs;

import android.content.Context;
import android.support.v7.widget.RecyclerView;
import android.text.Layout;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.ViewGroup;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.util.Collections;
import java.util.List;

import chuckcoughlin.sb.assistant.R;


/**
 * This a link between a RecyclerView and the data backstop.
 * Each element in the list is a string, a log message.
 */

public class RecyclerAdapter extends RecyclerView.Adapter<LogViewHolder> {
    private static final String CLSS = RecyclerAdapter.class.getSimpleName();
    private int nItems;
    private List<String> data = Collections.emptyList();

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
        LayoutInflater inflater = LayoutInflater.from(context);
        boolean shouldAttachToParent = false;

        // create a new view
        LinearLayout layout = (LinearLayout)inflater.inflate(R.layout.log_item,parent,shouldAttachToParent);

        LogViewHolder holder = new LogViewHolder(layout);
        return holder;
    }

    /**
     *
     * @param holder the viewholder that should be updated at the given position
     * @param position row that should be updated
     */
    @Override
    public void onBindViewHolder(LogViewHolder holder, int position) {
        //String message = data.get(position);
        LinearLayout layout = (LinearLayout)holder.itemView;
        //((TextView)layout.getChildAt(position)).setText("");
    }

    @Override
    public int getItemCount() {
        return nItems;
    }
}