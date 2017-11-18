/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;

import chuckcoughlin.sb.assistant.R;

/**
 * This fragment presents a static "cover" with no dynamic content.
 */

public class CoverFragment extends BasicAssistantFragment {

    // Inflate the view. It holds a the camera live image
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_cover, container, false);
        TextView label = (TextView) view.findViewById(R.id.fragmentCoverText);
        label.setText("@string/fragmentCoverLabel");

        ImageView imageView = (ImageView) view.findViewById(R.id.fragmentCoverImage);
        imageView.setImageResource(R.drawable.turtlebot3);
        return view;
    }
}
