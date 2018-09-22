/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.logs;

import android.app.Activity;
import android.content.Context;
import android.support.transition.TransitionManager;
import android.support.v7.widget.RecyclerView;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.text.SimpleDateFormat;
import java.util.Date;

import chuckcoughlin.sb.assistant.R;
import rosgraph_msgs.Log;


/**
 * This a link between a RecyclerView and the data backstop.
 * Each element in the list is a string, a log message.
 */

public class LogRecyclerAdapter extends RecyclerView.Adapter<LogViewHolder> implements LogListObserver {
    private static final String CLSS = LogRecyclerAdapter.class.getSimpleName();
    private static final String LEVEL_1 = "DEBUG";
    private static final String LEVEL_2 = "INFO";
    private static final String LEVEL_4 = "WARN";
    private static final String LEVEL_8 = "ERROR";
    private static final int MESSAGE_LEN = 45;
    private static final int SOURCE_LEN = 15;
    private static final int LOG_MSG_HEIGHT = 75;
    private static final int LOG_MSG_HEIGHT_EXPANDED = 225;
    private final SBLogManager logManager;
    private SimpleDateFormat dateFormatter = new SimpleDateFormat("HH:mm:ss.SSS");
    private int expandedPosition = -1;
    private Activity mainActivity = null;
    private RecyclerView recyclerView = null;
    private Context context = null;

    /**
     * Adapter between the recycler and data source for log messages
     */
    public LogRecyclerAdapter() {
        logManager = SBLogManager.getInstance();
        logManager.addObserver(this);
    }

    @Override
    public void onAttachedToRecyclerView(RecyclerView view) {
        this.recyclerView = view;
    }
    @Override
    public LogViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
        context = parent.getContext();
        LayoutInflater inflater = LayoutInflater.from(context);
        boolean shouldAttachToParent = false;

        // create a new view - set the item height.
        LinearLayout layout = (LinearLayout)inflater.inflate(R.layout.log_item,parent,shouldAttachToParent);
        LogViewHolder holder = new LogViewHolder(layout);
        return holder;
    }

    /**
     * Change the views depending on whether or not the item is selected.
     * In an expanded view some elements get re-used. The message is on its own line.
     * @param holder the viewholder that should be updated at the given position
     * @param position row that should be updated
     */
    @Override
    public void onBindViewHolder(LogViewHolder holder, int position) {
        boolean expand = (position==expandedPosition);
        Log msg = logManager.getLogAtPosition(position);  // Checks index bounds
        if( msg==null ) {
            android.util.Log.w(CLSS,String.format("Null log holder at %d",position));
            return;
        }
        // The timestamp is always the same
        TextView timestampView  = holder.getTimestampView();
        int secsFromEpoch = msg.getHeader().getStamp().secs;
        int msecs = msg.getHeader().getStamp().nsecs/1000;
        Date tstamp = new Date(secsFromEpoch*1000+msecs);
        String dt = dateFormatter.format(tstamp);
        timestampView.setText(dt);

        // In expanded mode the source is the level
        TextView sourceView  = holder.getSourceView();
        String source = msg.getName();
        if( expand ) {
            if(msg.getLevel()==1) sourceView.setText(LEVEL_1);
            else if(msg.getLevel()==2) sourceView.setText(LEVEL_2);
            else if(msg.getLevel()==4) sourceView.setText(LEVEL_4);
            else sourceView.setText(LEVEL_8);
        }
        else {
            // Truncate source to 16 char
            if( source.length()>SOURCE_LEN) source = source.substring(0,SOURCE_LEN);
            sourceView.setText(source);
        }

        // In expanded mode, the message is the source (node-name).
        TextView messageView  = holder.getMessageView();
        String msgText = msg.getMsg().trim();
        if( expand ) {
            messageView.setText(source);
        }
        else {
            if( msgText.length()>MESSAGE_LEN) msgText = msgText.substring(0,MESSAGE_LEN);
            messageView.setText(msgText);
        }


        TextView detailView = holder.getDetailView();
        ViewGroup.LayoutParams params = holder.itemView.getLayoutParams();
        if( expand ) {
            detailView.setText(msgText);
            detailView.setVisibility(View.VISIBLE);
            holder.itemView.setActivated(false);
            params.height=LOG_MSG_HEIGHT_EXPANDED;
        }
        else {
            detailView.setVisibility(View.GONE);
            holder.itemView.setActivated(true);
            params.height=LOG_MSG_HEIGHT;
        }
        holder.itemView.setLayoutParams(params);
        holder.itemView.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                expandedPosition = expand ? -1:position;
                TransitionManager.beginDelayedTransition(recyclerView);
                notifyDataSetChanged();
            }
        });
    }



    @Override
    public int getItemCount() {return logManager.getLogs().size(); }

    @Override
    public void onDetachedFromRecyclerView(RecyclerView recyclerView) {
        this.recyclerView = null;
        this.context = null;
    }


    // ============================= LogListObserver ===========================

    /**
     * A log message has been appended to the visible list.
     */
    @Override
    public void notifyListAppended() {
        final int size = getItemCount();
        if( context!=null ) {
            Activity activity = (Activity)context;
            activity.runOnUiThread(new Runnable() {
                public void run() {
                    notifyItemRangeInserted(size-1,1);
                }
            });
        }
    }

    /**
     * The entire list has been cleared. We assume this originates from a button
     * and is on the UI thread.
     */
    @Override
    public void notifyListCleared() {
        int size = getItemCount();
        notifyDataSetChanged();
    }
}