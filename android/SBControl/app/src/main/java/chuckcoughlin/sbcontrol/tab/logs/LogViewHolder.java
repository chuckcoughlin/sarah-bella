/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */
package chuckcoughlin.sbcontrol.tab.logs;

import android.support.v7.widget.RecyclerView;
import android.view.View;
import android.widget.TextView;

import chuckcoughlin.sbcontrol.R;

/**
 * Facilitates data transfer by the Recycler view.
 */

public class LogViewHolder extends RecyclerView.ViewHolder {
    private TextView itemView;

    public LogViewHolder(View containerView) {
        super(containerView);
        itemView = (TextView)containerView.findViewById(R.id.logs_recycle_item);
    }

    void bind(int listIndex) {
        itemView.setText(String.valueOf(listIndex));

    }
}
