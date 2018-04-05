/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.logs;

import android.support.v7.widget.RecyclerView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;
import android.widget.TextView;

import chuckcoughlin.sb.assistant.R;

/**
 * Facilitates data transfer by the Recycler view.
 */

public class LogViewHolder extends RecyclerView.ViewHolder {

    /**
     * The ViewGroup is actually a LinearLayout holding text
     * views:
     *    - timestamp
     *    - source
     *    - message
     * @param v the current view
     */
    public LogViewHolder(ViewGroup v) {
        super(v);
        this.setIsRecyclable(true);
    }

    public TextView getTimestampView() { return (TextView) ((ViewGroup)itemView).getChildAt(0); }
    public TextView getSourceView() { return (TextView) ((ViewGroup)itemView).getChildAt(1); }
    public TextView getMessageView() { return (TextView) ((ViewGroup)itemView).getChildAt(2); }

}
