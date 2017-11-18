/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;

import chuckcoughlin.sb.assistant.R;

/**
 * This fragment handles robot control during its SLAM
 * mapping sequence.
 */

public class SLAMFragment extends BasicAssistantFragment {

    // Inflate the view. It holds a the camera live image
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_slam, container, false);
        TextView label = (TextView) view.findViewById(R.id.fragmentSLAMText);
        label.setText("@string/fragmentSLAMLabel");

        ImageView imageView = (ImageView) view.findViewById(R.id.fragmentCameraImage);
        imageView.setImageResource(R.drawable.turtlebot3);
        return view;
    }
}
