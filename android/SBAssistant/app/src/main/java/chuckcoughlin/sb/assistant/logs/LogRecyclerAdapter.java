/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.logs;

import android.content.Context;
import android.support.v7.widget.RecyclerView;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;
import android.widget.TextView;

import chuckcoughlin.sb.assistant.R;
import rosgraph_msgs.Log;


/**
 * This a link between a RecyclerView and the data backstop.
 * Each element in the list is a string, a log message.
 */

public class LogRecyclerAdapter extends RecyclerView.Adapter<LogViewHolder> implements LogListObserver {
    private static final String CLSS = LogRecyclerAdapter.class.getSimpleName();
    private final SBLogManager logManager;

    /**
     * Adapter between the recycler and data source for log messages
     */
    public LogRecyclerAdapter() {
        logManager = SBLogManager.getInstance();
        logManager.addObserver(this);

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
        Log msg = logManager.getLogAtPosition(position);
        TextView timestampView  = holder.getTimestampView();
        timestampView.setText(msg.getHeader().getStamp().toString());
        TextView sourceView  = holder.getSourceView();
        sourceView.setText(msg.getName());
        TextView messageView  = holder.getMessageView();
        messageView.setText(msg.getMsg());
    }

    @Override
    public int getItemCount() {
        return logManager.getLogs().size();
    }

    // ============================= LogListObserver ===========================
    @Override
    public void notifyLogAppended() {
        this.notifyItemInserted(getItemCount()-1);
    }

    /**
     * It will always be the first log in the list
     * that is removed.
     */
    @Override
    public void notifyLogRemoved() {
        this.notifyItemRangeRemoved(0,1);
    }
}