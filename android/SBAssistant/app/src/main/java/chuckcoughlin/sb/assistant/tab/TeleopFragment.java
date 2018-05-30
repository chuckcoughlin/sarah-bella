/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * @See https://github.com/fcrisciani/android-speech-recognition/
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.ToggleButton;

import org.ros.android.view.DirectControlstickView;
import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBApplicationStatusListener;
import chuckcoughlin.sb.assistant.ros.SBApplicationManager;
import chuckcoughlin.sb.assistant.ros.SBRobotManager;
import ros.android.util.TabletApplication;

/**
 * This fragment handles manual robot control. It publishes Twist messages
 * and listens to ObstacleDistance, not letting the robot crash into something in front of it.
 * The publisher and subscriber are all internal to the virtual joystick.
 *
 * OFFLINE is a way of testing the widget and speech functions with the robot offline.
 */

public class TeleopFragment extends BasicAssistantFragment implements SBApplicationStatusListener,
                                                                    RecognitionListener {
    private static final String CLSS = "TeleopFragment";
    private static final boolean OFFLINE = true;
    private SBApplicationManager applicationManager;
    private DirectControlstickView joystick = null;
    private SpeechRecognizer sr = null;
    private ToggleButton speechToggle = null;


        // Inflate the view. It displays a virtual joystick
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,Bundle savedInstanceState) {
        this.applicationManager = SBApplicationManager.getInstance();
        View view = inflater.inflate(R.layout.fragment_teleops, container, false);
        TextView label = view.findViewById(R.id.fragmentTeleopsText);
        label.setText(R.string.fragmentTeleopLabel);
        speechToggle = view.findViewById(R.id.speech_toggle);
        speechToggle.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                speechToggleClicked();
            }
        });
        speechToggle.setEnabled(false);

        joystick = (DirectControlstickView)view.findViewById(R.id.virtual_joystick);
        if( !OFFLINE ) {
            joystick.setTopicNames("/sb_serve_twist_command", "/sb_obstacle_distance");   // Service & subscription topics
            applicationManager.addListener(this);
        }
        else {
            speechToggle.setEnabled(true);
            joystick.onStart(null);
        }

        sr = SpeechRecognizer.createSpeechRecognizer(getActivity());
        sr.setRecognitionListener(this);
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
            speechToggle.setEnabled(true);
        }
    }

    public void applicationShutdown(String appName) {
        if (appName.equalsIgnoreCase(SBConstants.APPLICATION_TELEOP)) {
            Log.i(CLSS, String.format("applicationShutdown"));
            joystick.onShutdownComplete();
        }
        speechToggle.setEnabled(false);
        if(sr!=null) {
            sr.stopListening();
            sr.destroy();
        }
        sr = null;
    }

    //  Start/stop toggle for speech entry clicked. This button is only enabled when
    // an application is running. The initial state is OFF.
    public void speechToggleClicked() {
        if( speechToggle.isChecked() ) {
            Log.i(CLSS,"speech toggle OFF");
            sr.stopListening();
        }
        else {
            Log.i(CLSS,"speech toggle ON");
            // Start speech recognition
            Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
            //Specify the calling package to identify your application
            intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE,getClass().getPackage().getName());
            //Give a hint to the recognizer about what the user is going to say
            intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
            //specify the max number of results
            intent.putExtra(RecognizerIntent.EXTRA_MAX_RESULTS,10);
            //User of SpeechRecognizer to "send" the intent.
            sr.startListening(intent);
            Log.i(CLSS,"Intent sent to SpeechRecognizer");
        }

    }
    // ========================================= RecognitionListener ============================
    public void onReadyForSpeech(Bundle params)  {
        Log.i(CLSS, "onReadyForSpeech");
    }
    public void onBeginningOfSpeech(){
        Log.i(CLSS, "onBeginningOfSpeech");
    }
    public void onRmsChanged(float rmsdB){
        Log.i(CLSS, "onRmsChanged");
    }
    public void onBufferReceived(byte[] buffer)  {
        Log.i(CLSS, "onBufferReceived");
    }
    public void onEndOfSpeech()  {
        Log.i(CLSS, "onEndofSpeech");
    }
    public void onError(int error)  {
        Log.i(CLSS,  String.format("SpeechRecognition: ERROR (%d) ",error));
    }
    public void onResults(Bundle results) {

        Log.d(CLSS, "onResults " + results);
        // Fill the list view with the strings the recognizer thought it could have heard, there should be 5, based on the call
        ArrayList<String> matches = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
        //display results.
        for (int i = 0; i < matches.size(); i++) {
            Log.i(CLSS, "result " + matches.get(i));
        }

    }
    public void onPartialResults(Bundle partialResults)
    {
        Log.i(CLSS, "onPartialResults");
    }
    public void onEvent(int eventType, Bundle params)
    {
        Log.i(CLSS, "onEvent " + eventType);
    }

}
