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

import org.ros.android.view.AudioCompassView;
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;

import java.net.URI;
import java.net.URISyntaxException;

import audio_locator.AudioLocation;
import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.AbstractMessageListener;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBApplicationManager;
import chuckcoughlin.sb.assistant.ros.SBApplicationStatusListener;
import chuckcoughlin.sb.assistant.ros.SBRobotManager;
import audio_locator.AudioLocation;
/**
 * Display the source of audio signals.
 */

public class AudioFragment extends BasicAssistantFragment implements SBApplicationStatusListener {
    private static final String CLSS = "AudioFragment";
    private AudioListener audioListener = null;
    private AudioCompassView compass = null;
    private SBApplicationManager applicationManager;

    // Inflate the view. It displays an audio compass
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,Bundle savedInstanceState) {
        this.applicationManager = SBApplicationManager.getInstance();
        audioListener = new AudioListener();
        View view = inflater.inflate(R.layout.fragment_audio, container, false);
        TextView label = view.findViewById(R.id.fragmentAudioText);


        compass = (AudioCompassView)view.findViewById(R.id.audio_compass);

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
        MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
        applicationManager.addListener(this);
        return view;
    }

    @Override
    public void onDestroyView() {
        Log.i(CLSS, "onDestroyView");
        applicationShutdown(SBConstants.APPLICATION_TELEOP);
    }

    // ============================= Audio Location Message Listener ===========================
    private class AudioListener extends AbstractMessageListener<AudioLocation> {
        public AudioListener() {
            super(audio_locator.AudioLocation._TYPE);
        }

        // If we are too close, shut things down.
        @Override
        public void onNewMessage(audio_locator.AudioLocation message) {
            getActivity().runOnUiThread(new Runnable() {
                public void run() {
                    compass.onAudioMessage(message);
                }
            });
        }
    }

    // ========================================= SBApplicationStatusListener ============================
    // This may be called immediately on establishment of the listener. It will not get called
    // if the robot is OFFLINE.  Inform the obstacle detector of the width of the robot.
    public void applicationStarted(String appName) {
        Log.i(CLSS, String.format("applicationStarted: %s ...", appName));

        if (appName.equalsIgnoreCase(SBConstants.APPLICATION_TELEOP)) {
            ConnectedNode node = applicationManager.getCurrentApplication().getConnectedNode();
            if (node != null) {
                new Thread(new Runnable() {
                    public void run() {
                        try {
                            SBRobotManager robotManager = SBRobotManager.getInstance();
                            String uriString = robotManager.getRobot().getRobotId().getMasterUri();
                            URI masterUri = new URI(uriString);
                            audioListener.subscribe(node,"/sb_audio_location");
                        }
                        catch(URISyntaxException uriex) {
                            Log.e(CLSS,String.format("URI Syntax Exception setting parameter (%s)",uriex.getLocalizedMessage()));
                        }
                        catch (Throwable ex) {
                            Log.e(CLSS, "Exception while creating audio listener ", ex);
                        }
                    }
                }).start();
            }
            else {
                Log.i(CLSS, String.format("applicationStarted: %s has no connected node", appName));
            }

        }
    }
    public void applicationShutdown(String appName) {
        if (appName.equalsIgnoreCase(SBConstants.APPLICATION_TELEOP)) {
            Log.i(CLSS, String.format("applicationShutdown"));
        }
    }
}
