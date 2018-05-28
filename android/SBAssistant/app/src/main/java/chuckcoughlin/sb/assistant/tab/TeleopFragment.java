/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import org.ros.android.view.DirectControlstickView;
import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBApplicationStatusListener;
import chuckcoughlin.sb.assistant.ros.SBApplicationManager;
import chuckcoughlin.sb.assistant.ros.SBRobotManager;

/**
 * This fragment handles manual robot control. It publishes Twist messages
 * and listens to Odometry. The publisher and subscriber are all internal to
 * the virtual joystick.
 */

public class TeleopFragment extends BasicAssistantFragment implements SBApplicationStatusListener {
    private static final String CLSS = "TeleopFragment";

    private SBApplicationManager applicationManager;
    private DirectControlstickView joystick = null;

    // Inflate the view. It displays a virtual joystick
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,Bundle savedInstanceState) {
        this.applicationManager = SBApplicationManager.getInstance();
        View view = inflater.inflate(R.layout.fragment_teleops, container, false);
        TextView label = view.findViewById(R.id.fragmentTeleopsText);
        label.setText(R.string.fragmentTeleopLabel);
        joystick = (DirectControlstickView)view.findViewById(R.id.virtual_joystick);
        joystick.setTopicNames("/sb_serve_twist_command","/sb_teleop");   // Publish & subscribe topics
        applicationManager.addListener(this);
        return view;
    }

    @Override
    public void onDestroyView() {
        Log.i(CLSS, "onDestroyView");
        applicationShutdown(SBConstants.APPLICATION_TELEOP);
        super.onDestroyView();
    }

    // ========================================= SBApplicationStatusListener ============================
    // This may be called immediately on establishment of the listener.
    // Inform the obstacle detector of the width of the robot.
    public void applicationStarted(String appName) {
        Log.i(CLSS, String.format("applicationStarted: %s ...", appName));
        if (appName.equalsIgnoreCase(SBConstants.APPLICATION_TELEOP)) {
            ConnectedNode node = applicationManager.getCurrentApplication().getConnectedNode();
            if (node != null) {
                joystick.onStart(node);
                new Thread(new Runnable() {
                    public void run() {
                        try {
                            SBRobotManager robotManager = SBRobotManager.getInstance();
                            String uriString = robotManager.getRobot().getRobotId().getMasterUri();
                            URI masterUri = new URI(uriString);
                            ParameterClient paramClient = new ParameterClient(new NodeIdentifier(GraphName.of("/TeleopFragment"),masterUri),masterUri);
                            paramClient.setParam(GraphName.of(SBConstants.ROS_WIDTH_PARAM),SBConstants.SB_ROBOT_WIDTH);
                        }
                        catch(URISyntaxException uriex) {
                            Log.e(CLSS,String.format("URI Syntax Exception setting parameter (%s)",uriex.getLocalizedMessage()));
                        }
                    }
                });
            }
            else {
                Log.i(CLSS, String.format("applicationStarted: %s has no connected node", appName));
            }
        }
    }

    public void applicationShutdown(String appName) {
        if (appName.equalsIgnoreCase(SBConstants.APPLICATION_TELEOP)) {
            Log.i(CLSS, String.format("applicationShutdown"));
            joystick.onShutdownComplete();
        }
    }
}
