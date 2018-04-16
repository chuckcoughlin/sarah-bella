/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.logs;

import android.app.Activity;
import android.app.Application;
import android.content.Context;
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
    private final SBLogManager logManager;
    private SimpleDateFormat dateFormatter = new SimpleDateFormat("HH:mm:ss.SSS");
    private int expandedPosition = -1;
    private Activity mainActivity = null;
    private Context context = null;

    /**
     * Adapter between the recycler and data source for log messages
     */
    public LogRecyclerAdapter() {
        logManager = SBLogManager.getInstance();
        logManager.addObserver(this);
    }

    @Override
    public LogViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
        context = parent.getContext();
        LayoutInflater inflater = LayoutInflater.from(context);
        boolean shouldAttachToParent = false;

        // create a new view - set the item height.
        LinearLayout layout = (LinearLayout)inflater.inflate(R.layout.log_item,parent,shouldAttachToParent);
        ViewGroup.LayoutParams params = layout.getLayoutParams();
        params.height = 75;
        layout.setLayoutParams(params);
        LogViewHolder holder = new LogViewHolder(layout);
        return holder;
    }

    /**
     * Change the views depending on whether or not the item is selected.
     * In an expanded each element gets its own line (TBD).
     * @param holder the viewholder that should be updated at the given position
     * @param position row that should be updated
     */
    @Override
    public void onBindViewHolder(LogViewHolder holder, int position) {
        Log msg = logManager.getLogAtPosition(position);
        TextView levelView  = holder.getLevelView();
        if(msg.getLevel()==1) levelView.setText(LEVEL_1);
        else if(msg.getLevel()==1) levelView.setText(LEVEL_2);
        else if(msg.getLevel()==1) levelView.setText(LEVEL_4);
        else levelView.setText(LEVEL_8);
        levelView.setVisibility(View.INVISIBLE);  // Don't display for now

        TextView timestampView  = holder.getTimestampView();
        int secsFromEpoch = msg.getHeader().getStamp().secs;
        int msecs = msg.getHeader().getStamp().nsecs/1000;
        Date tstamp = new Date(secsFromEpoch*1000+msecs);
        String dt = dateFormatter.format(tstamp);
        timestampView.setText(dt);
        // Truncate source to 16 chars
        TextView sourceView  = holder.getSourceView();
        String source = msg.getName();
        if( source.length()>SOURCE_LEN) source = source.substring(0,SOURCE_LEN);
        sourceView.setText(source);
        // May be multiple lines
        TextView messageView  = holder.getMessageView();
        String msgText = msg.getMsg().trim();
        if( msgText.length()>SOURCE_LEN) msgText = msgText.substring(0,MESSAGE_LEN);
        messageView.setText(msgText);

        //final boolean isExpanded = (position==expandedPosition);
        //holder.details.setVisibility(isExpanded?View.VISIBLE:View.GONE);
        //holder.itemView.setActivated(isExpanded);
    }

    @Override
    public int getItemCount() {
        return logManager.getLogs().size();
    }

    @Override
    public void onDetachedFromRecyclerView(RecyclerView recyclerView) {
        this.context = null;
    }

    // ============================= LogListObserver ===========================
    @Override
    public void notifyLogAppended() {
        final int count = getItemCount();
        if( context!=null ) {
            Activity activity = (Activity)context;
            activity.runOnUiThread(new Runnable() {
                public void run() {
                    notifyItemInserted(count);
                }
            });
        }
    }

    /**
     * It will always be the first log in the list
     * that is removed.
     */
    @Override
    public void notifyLogChanged() {
        Activity activity = (Activity)context;
        activity.runOnUiThread(new Runnable() {
            public void run() {
                notifyDataSetChanged();
            }
        });
    }
    /**
     * It will always be the first log in the list
     * that is removed.
     */
    @Override
    public void notifyLogRemoved() {
        Activity activity = (Activity)context;
        activity.runOnUiThread(new Runnable() {
            public void run() {
                notifyItemRemoved(0);
            }
        });
    }
}