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
 * This fragment handles manual robot control
 */

public class TeleopFragment extends BasicAssistantFragment {

    // Inflate the view. It displays a virtual joystick
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_teleops, container, false);
        TextView label = view.findViewById(R.id.fragmentTeleopsText);
        label.setText(R.string.teleops_title);

        return view;
    }
}
