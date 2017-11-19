/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */
package chuckcoughlin.sb.assistant.logs;

import android.support.v7.widget.RecyclerView;
import android.view.View;
import android.widget.TextView;

import chuckcoughlin.sb.assistant.R;

/**
 * Facilitates data transfer by the Recycler view.
 */

public class LogViewHolder extends RecyclerView.ViewHolder {

    /**
     * The item view is actually a LinearLayout
     * @param iView contains the list of text views
     */
    public LogViewHolder(View iView) {
        super(iView);
    }


}
