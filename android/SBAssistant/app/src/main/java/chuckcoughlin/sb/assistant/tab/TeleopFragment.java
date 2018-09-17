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
import android.widget.RadioGroup;
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
import teleop_service.BehaviorCommandRequest;
import teleop_service.BehaviorCommandResponse;
import teleop_service.ObstacleDistance;
import teleop_service.TeleopStatus;
import teleop_service.TwistCommandRequest;
import teleop_service.TwistCommandResponse;

/**
 * This fragment handles manual robot control. It publishes Twist messages
 * and listens to ObstacleDistance, not letting the robot crash into something in front of it.
 *
 * OFFLINE is a way of testing the widget and speech functions with the robot offline.
 */

public class TeleopFragment extends BasicAssistantFragment implements SBApplicationStatusListener,
                                                ServiceResponseListener<BehaviorCommandResponse>,
                                                TwistCommandController, RecognitionListener,
                                                TextToSpeech.OnInitListener, AdapterView.OnItemSelectedListener,
                                                RadioGroup.OnCheckedChangeListener  {
    private static final String CLSS = "TeleopFragment";

    // ================================== Timing Constants ===========================
    private static final double DELTA_ANGLE               = 0.2;  // Max normalized angle change in a step was 0.02
    private static final double DELTA_VELOCITY            = 0.2;  // Max normalized velocity change in a step 0.02
    private static final double MIN_OBSTACLE_DISTANCE     = 0.40;
    private static final long OFFLINE_TIMER_PERIOD = 3000;  // ~msecs
    private static final long TIMER_PERIOD         = 100;   // ~msecs

    private SBApplicationManager applicationManager;
    private TwistCommandRequest currentRequest = null;    // Most recent state
    private TwistCommandRequest targetRequest = null;     // Desired state
    private TwistCommandInterpreter interpreter = new TwistCommandInterpreter(this);
    private DirectControlstickView joystick = null;
    private boolean isOnline = true;    // Offline is a debug mode, no commands sent to robot
    private double obstacleDistance = Double.MAX_VALUE;
    private ServiceClient<BehaviorCommandRequest, BehaviorCommandResponse> behaviorServiceClient = null;
    private ServiceClient<TwistCommandRequest, TwistCommandResponse> twistServiceClient = null;
    private ServiceTimer serviceTimer = null;       // Publish velocity commands at a constant rate.
    private DistanceListener distanceListener = null;
    private TeleopStatusListener statusListener = null;
    private int language = TwistCommandController.ENGLISH;
    private SpeechRecognizer sr = null;
    private ObstacleErrorAnnunciator speechSynthesizer = null;
    private ToggleButton onlineToggle = null;
    private ToggleButton speechToggle = null;
    private RadioGroup behaviorGroup = null;
    private TextView angularVelocityView = null;
    private TextView linearVelocityView = null;
    private TextView teleopStatusView = null;
    ParameterClient paramClient = null;


    // Inflate the view. It displays a virtual joystick
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,Bundle savedInstanceState) {
        this.applicationManager = SBApplicationManager.getInstance();
        distanceListener = new DistanceListener();
        statusListener = new TeleopStatusListener();
        View view = inflater.inflate(R.layout.fragment_teleops, container, false);
        TextView label = view.findViewById(R.id.fragmentTeleopsText);
        label.setText(R.string.fragmentTeleopLabel);
        behaviorGroup = view.findViewById(R.id.behaviorRadioGroup);
        behaviorGroup.setOnCheckedChangeListener(this);
        onlineToggle = view.findViewById(R.id.online_toggle);
        onlineToggle.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onlineToggleClicked();
            }
        });
        onlineToggle.setChecked(isOnline);

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
                R.array.languages_array, R.layout.spinner_item);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        languageSpinner.setAdapter(adapter);

        joystick = (DirectControlstickView)view.findViewById(R.id.virtual_joystick);
        joystick.setController(this);
        angularVelocityView = (TextView)view.findViewById(R.id.velocity_field);
        linearVelocityView = (TextView)view.findViewById(R.id.angular_velocity_field);
        teleopStatusView = (TextView)view.findViewById(R.id.teleops_status_field);

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
        MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
        targetRequest = messageFactory.newFromType(TwistCommandRequest._TYPE);
        targetRequest.setLinearX(0.0);  // Stopped
        targetRequest.setAngularZ(0.0);

        applicationManager.addListener(this);
        speechToggle.setEnabled(true);
        currentRequest = messageFactory.newFromType(TwistCommandRequest._TYPE);
        currentRequest.setLinearX(0.0);  // Stopped
        currentRequest.setAngularZ(0.0);
        serviceTimer = new ServiceTimer(isOnline,this);

        sr = SpeechRecognizer.createSpeechRecognizer(getActivity());
        sr.setRecognitionListener(TeleopFragment.this);
        speechSynthesizer = new ObstacleErrorAnnunciator(getActivity(),this);

        return view;
    }

    @Override
    public void onDestroyView() {
        Log.i(CLSS, "onDestroyView");
        applicationShutdown(SBConstants.APPLICATION_TELEOP);
        if( speechSynthesizer!=null) speechSynthesizer.shutdown();
        super.onDestroyView();
        if (sr != null) {
            sr.stopListening();
            sr.destroy();
        }
        sr = null;
        if( serviceTimer!=null ) {
            serviceTimer.cancel();
            serviceTimer.purge();
            serviceTimer= null;
        }
    }

    // ======================================== OnCheckedChangeListener ===============================
    public void onCheckedChanged(RadioGroup group, int checkedId) {
        setBehavior(checkedId);
    }
    // Inform the behavior service of the new desired behavior.
    private void setBehavior(int checkedId) {
        // checkedId is true if the RadioButton is selected
        if( behaviorServiceClient!=null && isOnline ) {
            BehaviorCommandRequest request = behaviorServiceClient.newMessage();
            switch (checkedId) {
                case R.id.joystick:
                    request.setBehavior(SBConstants.SB_BEHAVIOR_JOYSTICK);
                    break;
                case R.id.come:
                    request.setBehavior(SBConstants.SB_BEHAVIOR_COME);
                    break;
                case R.id.follow:
                    request.setBehavior(SBConstants.SB_BEHAVIOR_FOLLOW);
                    break;
                case R.id.park:
                    request.setBehavior(SBConstants.SB_BEHAVIOR_PARK);
                    break;
                default:
                    Log.i(CLSS, String.format("setBehavior: Unrecognized selection"));
                    request.setBehavior(SBConstants.SB_BEHAVIOR_JOYSTICK);
            }

            behaviorServiceClient.call(request,this);
        }
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
                            paramClient = new ParameterClient(new NodeIdentifier(GraphName.of("/TeleopFragment"),masterUri),masterUri);
                            paramClient.setParam(GraphName.of(SBConstants.ROS_WIDTH_PARAM),SBConstants.SB_ROBOT_WIDTH);
                            behaviorServiceClient = node.newServiceClient("/sb_serve_behavior_command", teleop_service.BehaviorCommand._TYPE);
                            twistServiceClient = node.newServiceClient("/sb_serve_twist_command", teleop_service.TwistCommand._TYPE);
                            currentRequest = twistServiceClient.newMessage();
                            currentRequest.setLinearX(0.0);  // Stopped
                            currentRequest.setAngularZ(0.0);
                            distanceListener.subscribe(node,"/sb_obstacle_distance");
                            statusListener.subscribe(node,"/sb_teleop_status");
                            restartServiceTimer(isOnline);
                            setBehavior(behaviorGroup.getCheckedRadioButtonId());
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

        }
    }

    public void applicationShutdown(String appName) {
        if (appName.equalsIgnoreCase(SBConstants.APPLICATION_TELEOP)) {
            Log.i(CLSS, String.format("applicationShutdown"));

            getActivity().runOnUiThread(new Runnable() {
                public void run() {
                    speechToggle.setEnabled(false);
                }
            });
            if (serviceTimer != null && isOnline) {
                serviceTimer.cancel();
                serviceTimer.purge();
                serviceTimer = null;
            }

            if( speechSynthesizer!=null ) {
                speechSynthesizer.stop();
                speechSynthesizer = null;
            }
        }
    }
    //  Toggle the debug state. When "offline" we don't send messages to the robot.
    public void onlineToggleClicked() {
        isOnline = onlineToggle.isChecked();
        if( isOnline ) {
            Log.i(CLSS,"Mode is ONLINE");
        }
        else {
            Log.i(CLSS,"Mode is OFFLINE");
        }
        restartServiceTimer(isOnline);
    }

    private void restartServiceTimer(boolean online) {
        if( serviceTimer!=null) {
            serviceTimer.cancel();
            serviceTimer.purge();
        }
        serviceTimer = new ServiceTimer(online,this);
    }
    //  Start/stop toggle for speech entry cli cked. This button is only enabled when
    // an application is running. The initial state is OFF.
    public void speechToggleClicked() {
        if( speechToggle.isChecked() ) {
            Log.i(CLSS,"speech toggle ON");
            startRecognizer();
        }
        else {
            Log.i(CLSS,"speech toggle OFF");
            if(sr!=null) sr.stopListening();
        }
    }
    // start or restart recognizer if the speech recognizer button is checked
    private void startRecognizer() {
        if( !speechToggle.isChecked()) return;
        getActivity().runOnUiThread(new Runnable() {
            public void run() {
                if(sr!=null) sr.cancel();

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
                // On the Android device, settings, go to SBAssistant and enable the microphone
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
     * The actual command to the robot is performed on a timer.
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
        }
    }
    public TwistCommandRequest getCurrentRequest()  { return this.currentRequest; }
    public TwistCommandRequest getTargetRequest()   { return this.targetRequest;  }
    // Used to calibrate the speed in the UI.
    @Override
    public double getMaxSpeed() {
        return 0;
    }

    // ================================= ServiceResponseListener ====================
    @Override
    public void onSuccess(BehaviorCommandResponse behaviorCommandResponse) {
    }

    @Override
    public void onFailure(RemoteException e) {
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
        private boolean errorAnnunciated;      // Prevents repeated error announcements
        private boolean online;
        private TimerTask task;

        public ServiceTimer(boolean online,TwistCommandController c) {
            this.controller = c;
            final ServiceTimer thistimer = this;
            this.errorAnnunciated = false;
            this.task = new TimerTask() {
                @Override
                public void run() {
                    TwistCommandRequest current = controller.getCurrentRequest();
                    TwistCommandRequest target = controller.getTargetRequest();
                    if( obstacleDistance<MIN_OBSTACLE_DISTANCE && targetRequest.getLinearX()>0.  ) {
                        targetRequest.setLinearX(0.0);
                        targetRequest.setAngularZ(0.0);
                        Log.i(TAG,String.format("ServiceTimer: Too close %3.0f vs %3.0f",obstacleDistance,MIN_OBSTACLE_DISTANCE));
                        if( !errorAnnunciated && speechToggle.isChecked() && speechSynthesizer!=null ) {
                            speechSynthesizer.annunciateError(language,obstacleDistance);
                            errorAnnunciated = true;
                        }
                    }
                    else {
                        errorAnnunciated = false;
                    }
                    double currentX = current.getLinearX();
                    double currentZ = current.getAngularZ();
                    current.setLinearX(rampedVelocity(current,target));
                    double raz = rampedAngle(current,target);
                    current.setAngularZ(raz);
                    if (online) {
                        // Only call if there has been a change
                        if( currentX!=current.getLinearX() || currentZ!=current.getAngularZ()) {
                            if( twistServiceClient!=null ) {
                                twistServiceClient.call(current,thistimer);
                                if( getActivity()!=null ) {
                                    getActivity().runOnUiThread(new Runnable() {
                                        public void run() {
                                            linearVelocityView.setText(String.valueOf(current.getLinearX()));
                                            angularVelocityView.setText(String.valueOf(current.getAngularZ()));
                                        }
                                    });
                                }
                            }
                            Log.i(TAG, String.format("call: ONLINE, but twist service client is NULL"));
                        }
                    }
                    else {
                        Log.i(TAG, String.format("call: target: %3.2f %3.2f, current: %3.2f %3.2f", target.getLinearX(),target.getAngularZ(),
                                current.getLinearX(),current.getAngularZ()));

                    }
                    // Change our target angle by our increment. Aim for straight again.
                    target.setAngularZ(straighten(target));
                }
            };
            if( online ) {
                schedule(task,0,TIMER_PERIOD);
            }
            else {
                schedule(task,0,OFFLINE_TIMER_PERIOD);
            }
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
        private double straighten(TwistCommandRequest req) {
            double error = req.getAngularZ();
            if( Math.abs(error)<DELTA_ANGLE ) return 0.;
            else if(error<0.)  return req.getAngularZ() + DELTA_ANGLE;
            else return req.getAngularZ() - DELTA_ANGLE;
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
            //Log.i(CLSS,String.format("received ObstacleDistance = (%f)",obstacleDistance));
            if( obstacleDistance< SBConstants.SB_ROBOT_CLOSEST_APPROACH ) {
                // We know this will fail ...
                commandVelocity(targetRequest.getLinearX(),targetRequest.getAngularZ());
                if( behaviorGroup.getCheckedRadioButtonId()==R.id.joystick && getActivity()!=null  ) {
                    getActivity().runOnUiThread(new Runnable() {
                        public void run() {
                            teleopStatusView.setText(String.valueOf(obstacleDistance));
                        }
                    });
                }
            }
        }
    }
    // ============================= Teleop Status Message Listener ===========================
    private class TeleopStatusListener extends AbstractMessageListener<TeleopStatus> {
        public TeleopStatusListener() {
            super(teleop_service.TeleopStatus._TYPE);
        }

        // Simply display the status we get from the robot.
        @Override
        public void onNewMessage(teleop_service.TeleopStatus message) {
            Log.i(CLSS,String.format("Teleop Status message: (%s)",message.getStatus()));
            if( getActivity()!=null ) {
                getActivity().runOnUiThread(new Runnable() {
                    public void run() {
                        teleopStatusView.setText(message.getStatus());
                    }
                });
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