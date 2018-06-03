/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 * @See https://github.com/fcrisciani/android-speech-recognition/
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.content.Intent;
import android.os.Bundle;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.speech.tts.TextToSpeech;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.ToggleButton;

import org.ros.android.view.DirectControlstickView;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.message.MessageFactory;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.AbstractMessageListener;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.control.ObstacleErrorAnnunciator;
import chuckcoughlin.sb.assistant.control.TwistCommandController;
import chuckcoughlin.sb.assistant.control.TwistCommandInterpreter;
import chuckcoughlin.sb.assistant.ros.SBApplicationManager;
import chuckcoughlin.sb.assistant.ros.SBApplicationStatusListener;
import chuckcoughlin.sb.assistant.ros.SBRobotManager;
import teleop_service.ObstacleDistance;
import teleop_service.TwistCommandRequest;
import teleop_service.TwistCommandResponse;

/**
 * This fragment handles manual robot control. It publishes Twist messages
 * and listens to ObstacleDistance, not letting the robot crash into something in front of it.
 *
 * OFFLINE is a way of testing the widget and speech functions with the robot offline.
 */

public class TeleopFragment extends BasicAssistantFragment implements SBApplicationStatusListener,
                                                TwistCommandController, RecognitionListener,
                                                TextToSpeech.OnInitListener, AdapterView.OnItemSelectedListener  {
    private static final String CLSS = "TeleopFragment";

    // ================================== Timing Constants ===========================
    private static final double DELTA_ANGLE               = 0.02;  // Max normalized angle change in a step
    private static final double DELTA_VELOCITY            = 0.02;  // Max normalized velocity change in a step
    private static final double MIN_OBSTACLE_DISTANCE     = 20.;
    private static final long OFFLINE_TIMER_PERIOD = 2000;  // ~msecs
    private static final long TIMER_PERIOD         = 100;   // ~msecs


    private static final boolean OFFLINE = true;
    private SBApplicationManager applicationManager;
    private TwistCommandRequest currentRequest = null;    // Most recent state
    private TwistCommandRequest targetRequest = null;     // Desired state
    private TwistCommandInterpreter interpreter = new TwistCommandInterpreter(this);
    private DirectControlstickView joystick = null;
    private double obstacleDistance = Double.MAX_VALUE;
    private ServiceClient<TwistCommandRequest, TwistCommandResponse> serviceClient = null;
    private ServiceTimer serviceTimer = null;       // Publish velocity commands at a constant rate.
    private DistanceListener distanceListener = null;
    private boolean errorAnnunciated = false;      // Prevents repeated error announcements
    private int language = TwistCommandController.ENGLISH;
    private SpeechRecognizer sr = null;
    private ObstacleErrorAnnunciator speechSynthesizer = null;
    private ToggleButton speechToggle = null;


    // Inflate the view. It displays a virtual joystick
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,Bundle savedInstanceState) {
        this.applicationManager = SBApplicationManager.getInstance();
        distanceListener = new DistanceListener();
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
        Spinner languageSpinner = (Spinner)view.findViewById(R.id.languages_spinner);
        ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(getActivity(),
                R.array.languages_array, android.R.layout.simple_spinner_item);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        languageSpinner.setAdapter(adapter);

        joystick = (DirectControlstickView)view.findViewById(R.id.virtual_joystick);
        joystick.setController(this);

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
        MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
        targetRequest = messageFactory.newFromType(TwistCommandRequest._TYPE);
        targetRequest.setLinearX(0.0);  // Stopped
        targetRequest.setAngularZ(0.0);

        if( !OFFLINE ) {
            applicationManager.addListener(this);
        }
        else {
            speechToggle.setEnabled(true);
            Log.i(CLSS, String.format("onCreateView: starting in OFFLINE test mode"));
            currentRequest = messageFactory.newFromType(TwistCommandRequest._TYPE);
            currentRequest.setLinearX(0.0);  // Stopped
            currentRequest.setAngularZ(0.0);
            serviceTimer = new ServiceTimer(this);
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
    // This may be called immediately on establishment of the listener. It will not get called
    // if the robot is OFFLINE.  Inform the obstacle detector of the width of the robot.
    public void applicationStarted(String appName) {
        Log.i(CLSS, String.format("applicationStarted: %s ...", appName));
        final TwistCommandController controller = this;
        if (appName.equalsIgnoreCase(SBConstants.APPLICATION_TELEOP)) {
            ConnectedNode node = applicationManager.getCurrentApplication().getConnectedNode();
            if (node != null) {
                new Thread(new Runnable() {
                    public void run() {
                        try {
                            SBRobotManager robotManager = SBRobotManager.getInstance();
                            String uriString = robotManager.getRobot().getRobotId().getMasterUri();
                            URI masterUri = new URI(uriString);
                            ParameterClient paramClient = new ParameterClient(new NodeIdentifier(GraphName.of("/TeleopFragment"),masterUri),masterUri);
                            paramClient.setParam(GraphName.of(SBConstants.ROS_WIDTH_PARAM),SBConstants.SB_ROBOT_WIDTH);
                            serviceClient = node.newServiceClient("/sb_serve_twist_command", teleop_service.TwistCommand._TYPE);
                            currentRequest = serviceClient.newMessage();
                            currentRequest.setLinearX(0.0);  // Stopped
                            currentRequest.setAngularZ(0.0);
                            distanceListener.subscribe(node,"/sb_teleop");
                            serviceTimer = new ServiceTimer(controller);
                        }
                        catch(URISyntaxException uriex) {
                            Log.e(CLSS,String.format("URI Syntax Exception setting parameter (%s)",uriex.getLocalizedMessage()));
                        }
                        catch (ServiceNotFoundException snfe) {
                            Log.e(CLSS, String.format("Exception while creating service client (%s)", snfe.getLocalizedMessage()));
                        }
                        catch (XmlRpcTimeoutException tex) {
                            // With Bluetooth - UnresolvedAddressException creating service client.
                            Log.e(CLSS, "Exception while creating service client");
                        }
                        catch (Throwable ex) {
                            Log.e(CLSS, "Exception while creating parameter client ", ex);
                        }
                    }
                }).start();
            }
            else {
                Log.i(CLSS, String.format("applicationStarted: %s has no connected node", appName));
            }
            getActivity().runOnUiThread(new Runnable() {
                public void run() {
                    speechToggle.setEnabled(true);
                }
            });
            speechSynthesizer = new ObstacleErrorAnnunciator(getActivity(),this);
        }
    }

    public void applicationShutdown(String appName) {
        if (appName.equalsIgnoreCase(SBConstants.APPLICATION_TELEOP)) {
            Log.i(CLSS, String.format("applicationShutdown"));

            getActivity().runOnUiThread(new Runnable() {
                public void run() {
                    speechToggle.setEnabled(false);
                }
            });;
            if (serviceTimer != null) {
                serviceTimer.cancel();
                serviceTimer.purge();
                serviceTimer = null;
            }

            if (sr != null) {
                sr.stopListening();
                sr.destroy();
            }
            sr = null;
            if( speechSynthesizer!=null ) {
                speechSynthesizer.stop();
                speechSynthesizer = null;
            }
        }
    }

    //  Start/stop toggle for speech entry clicked. This button is only enabled when
    // an application is running. The initial state is OFF.
    public void speechToggleClicked() {
        if( speechToggle.isChecked() ) {
            Log.i(CLSS,"speech toggle ON");
            startRecognizer();
        }
        else {
            Log.i(CLSS,"speech toggle OFF");
            sr.stopListening();
        }
    }
    // start or restart recognizer if the speech recognizer button is checked
    private void startRecognizer() {
        if( !speechToggle.isChecked()) return;
        getActivity().runOnUiThread(new Runnable() {
            public void run() {
                sr.cancel();
                String locale =  "us-UK";
                if( language==RUSSIAN) locale =  "ru-RU";
                else if( language==FRENCH ) locale = "fr-FR";

                Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
                intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE,getClass().getPackage().getName());
                intent.putExtra(RecognizerIntent.EXTRA_PARTIAL_RESULTS,false);  // Partials are always empty
                //Give a hint to the recognizer about what the user is going to say
                intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
                intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, language);
                intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_PREFERENCE, language);
                intent.putExtra(RecognizerIntent.EXTRA_ONLY_RETURN_LANGUAGE_PREFERENCE, language);
                // Max number of results. This is three attempts at deciphering, not a 3-word limit.
                intent.putExtra(RecognizerIntent.EXTRA_MAX_RESULTS,3);
                sr.startListening(intent);
                Log.i(CLSS,"SpeechRecognizer: listening ...");
            }
        });
    }
    // ======================================= onInitListener ==================================
    @Override
    public void onInit(int status) {
        if( status!=TextToSpeech.SUCCESS ) {
            Log.w(CLSS,String.format("onInit: Error initializing speech synthesis (%d)",status));
            speechSynthesizer = null;   // Do not use
        }
    }
    // ========================================= RecognitionListener ============================
    public void onReadyForSpeech(Bundle params)  {
        //Log.i(CLSS, "onReadyForSpeech");
    }
    public void onBeginningOfSpeech(){
        //Log.i(CLSS, "onBeginningOfSpeech");
    }
    // Background level changed ...
    public void onRmsChanged(float rmsdB){
    }
    public void onBufferReceived(byte[] buffer)  {
        Log.i(CLSS, "onBufferReceived");
    }
    public void onEndOfSpeech()  {
        //Log.i(CLSS, "onEndofSpeech");
    }
    public void onError(int error)  {
        switch (error) {
            case SpeechRecognizer.ERROR_AUDIO:
                Log.i(CLSS,  String.format("SpeechRecognition: Audio recording error"));
                break;
            case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
                Log.i(CLSS,  String.format("SpeechRecognition: INSUFFICIENT PERMISSION - Enable microphone in app"));
                break;
            case SpeechRecognizer.ERROR_NO_MATCH:
                Log.i(CLSS,  String.format("SpeechRecognition: Error - no word match. Enunciate!"));
                startRecognizer();  // Try again
                break;
            case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
                Log.i(CLSS,  String.format("SpeechRecognition: Error - no speech input"));
                startRecognizer();  // Try again
                break;
            case SpeechRecognizer.ERROR_NETWORK:
                Log.i(CLSS,  String.format("SpeechRecognition: Error network"));
                break;
            case SpeechRecognizer.ERROR_CLIENT:
                Log.i(CLSS,  String.format("SpeechRecognition: Error - in client"));
                break;
            case SpeechRecognizer.ERROR_SERVER:
                Log.i(CLSS,  String.format("SpeechRecognition: Error - in server"));
                break;
            default:
                Log.i(CLSS,  String.format("SpeechRecognition: ERROR (%d) ",error));
        }

    }
    public void onResults(Bundle results) {
        //Log.i(CLSS, "onResults \n" + results);
        // Fill the list view with the strings the recognizer thought it could have heard, there should be 5, based on the call
        ArrayList<String> matches = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
        //display results. The zeroth result is usually the space-separated one.
        for (int i = 0; i < matches.size(); i++) {
            Log.i(CLSS, "result " + matches.get(i));
            if( interpreter.handleWordList(currentRequest,matches.get(i),language)) break;
        }
        startRecognizer();   // restart
    }
    public void onPartialResults(Bundle partialResults) {
        Log.i(CLSS, "onPartialResults");
    }
    public void onEvent(int eventType, Bundle params) {
        Log.i(CLSS, "onEvent " + eventType);
    }

    // ========================================= TwistCommandController ============================

    @Override
    /**
     * Request a new velocity / direction. This becomes the target.
     *
     * @param linearVelocityX  normalized linear velocity (-1 to 1).
     * @param angularVelocityZ normalized angular velocity (-1 to 1).
     */
    public void commandVelocity(double linearVelocityX,double angularVelocityZ) {
        //Log.i(CLSS, String.format("commandVelocity: %f %f", linearVelocityX, angularVelocityZ));
        if (targetRequest != null) {
            targetRequest.setLinearX(linearVelocityX);
            targetRequest.setLinearY(0);
            targetRequest.setLinearZ(0);
            targetRequest.setAngularX(0);
            targetRequest.setAngularY(0);
            targetRequest.setAngularZ(angularVelocityZ);

            if( obstacleDistance<MIN_OBSTACLE_DISTANCE && targetRequest.getLinearX()>0.  ) {
                targetRequest.setLinearX(0.0);
                targetRequest.setAngularZ(0.0);
                Log.i(CLSS,String.format("commandVelocity: Too close %3.0f vs %3.0f",obstacleDistance,MIN_OBSTACLE_DISTANCE));
                if( !errorAnnunciated && speechToggle.isChecked() && speechSynthesizer!=null ) {
                    speechSynthesizer.annunciateError(language,obstacleDistance);
                    errorAnnunciated = true;
                }
            }
            else {
                errorAnnunciated = false;
            }
        }
    }
    public TwistCommandRequest getCurrentRequest()  { return this.currentRequest; }
    public TwistCommandRequest getTargetRequest()   { return this.targetRequest;  }
    // Used to calibrate the speed in the UI.
    @Override
    public double getMaxSpeed() {
        return 0;
    }

    // ===================================== ServiceTimer ========================================
    /**
     * NEED TO VERIFY
     * The Turtlebot requires constant updates to keep it from shutting down. This provides a
     * constant stream of requests. Additionally this timer handles ramping between the current
     * state and target state both in terms of velocity and direction.
     */
    private class ServiceTimer extends Timer implements ServiceResponseListener<TwistCommandResponse> {
        private final static String TAG = "ServiceTimer";
        private final TwistCommandController controller;

        public ServiceTimer(TwistCommandController c) {
            this.controller = c;
            final ServiceTimer thistimer = this;
            long period = TIMER_PERIOD;
            if( OFFLINE ) period = OFFLINE_TIMER_PERIOD;
            TimerTask task = new TimerTask() {
                @Override
                public void run() {
                    TwistCommandRequest current = controller.getCurrentRequest();
                    TwistCommandRequest target = controller.getTargetRequest();
                    current.setLinearX(rampedVelocity(current,target));
                    current.setAngularZ(rampedAngle(current,target));

                    if (OFFLINE) {
                        Log.i(CLSS, String.format("call: %f %f", current.getLinearX(), current.getAngularZ()));
                    }
                    else {
                        Log.i(CLSS, String.format("call: %f %f", current.getLinearX(), current.getAngularZ()));
                        serviceClient.call(current,thistimer);
                    }
                }
            };
            schedule(task,0,period);
        }
        // =============================== ServiceResponseListener =====================================
        // Use these methods to handle service responses. In general, the responses have no useful information.
        // The framework requires that they be handled, however.
        @Override
        public void onSuccess(TwistCommandResponse response) {
            //Log.i(CLSS, String.format("SUCCESS: TwistCommandResponse (%s)",response.getMsg()));
        }

        @Override
        public void onFailure(RemoteException re) {
            Log.w(CLSS,String.format("Exception returned from service request (%s)",re.getLocalizedMessage()));
        }

        private double rampedAngle(TwistCommandRequest current,TwistCommandRequest target) {
            double error = current.getAngularX() - target.getAngularX();
            if( Math.abs(error)<DELTA_ANGLE ) return target.getAngularZ();
            else if(error<0.)  return current.getAngularZ() + DELTA_ANGLE;
            else return current.getAngularZ() - DELTA_ANGLE;
        }

        private double rampedVelocity(TwistCommandRequest current,TwistCommandRequest target) {
            double error = current.getLinearX() - target.getLinearX();
            //Log.i(CLSS, String.format("rampedVelocity: %f %3.2f %3.2f", error, current.getLinearX(),target.getLinearX()));
            if( Math.abs(error) < DELTA_VELOCITY ) return target.getLinearX();
            else if(error<0.)  return current.getLinearX() + DELTA_VELOCITY;
            else return current.getLinearX() - DELTA_VELOCITY;
        }
    }

    // ============================= Obstacle Distance Message Listener ===========================
    private class DistanceListener extends AbstractMessageListener<ObstacleDistance> {
        public DistanceListener() {
            super(teleop_service.ObstacleDistance._TYPE);
        }

        // If we are too close, shut things down.
        @Override
        public void onNewMessage(teleop_service.ObstacleDistance message) {
            obstacleDistance = message.getDistance();
            Log.i(CLSS,String.format("received ObstacleDistance = (%f)",obstacleDistance));
            if( obstacleDistance< SBConstants.SB_ROBOT_CLOSEST_APPROACH ) {
                commandVelocity(0.,0);
            }
        }
    }
    // ====================================== OnItemSelectedListener ===============================
    // Listener for the languages pull-down
    @Override
    public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
        this.language = position;
    }

    @Override
    public void onNothingSelected(AdapterView<?> parent) {
    }
}