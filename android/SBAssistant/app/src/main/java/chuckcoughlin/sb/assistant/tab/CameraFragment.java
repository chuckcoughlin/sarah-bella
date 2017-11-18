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
 * This fragment allows preusal of the robot's activity log.
 */

public class CameraFragment extends BasicAssistantFragment {

    // Inflate the view. It holds a the camera live image
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_camera, container, false);
        TextView label = (TextView) view.findViewById(R.id.fragmentCameraText);
        label.setText("@string/fragmentCameraLabel");

        ImageView imageView = (ImageView) view.findViewById(R.id.fragmentCameraImage);
        imageView.setImageResource(R.drawable.turtlebot);
        return view;
    }
}
