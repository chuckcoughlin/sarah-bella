/**
 * Copyright 2017 Charles Coughlin. All rights reserved.
 *  (MIT License)
 */

package chuckcoughlin.sb.assistant.tab;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.SeekBar;
import android.widget.TextView;

import org.ros.android.view.VerticalSeekBar;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.layer.LaserScanLayer;
import org.ros.android.view.visualization.layer.Layer;
import org.ros.node.ConnectedNode;

import java.util.ArrayList;
import java.util.List;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.AbstractMessageListener;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBApplicationStatusListener;
import chuckcoughlin.sb.assistant.ros.SBRosApplicationManager;
import sensor_msgs.LaserScan;
import tf.tfMessage;

/**
 * This fragment handles robot control during its SLAM
 * mapping sequence.
 */

public class LidarFragment extends BasicAssistantFragment implements SBApplicationStatusListener, SeekBar.OnSeekBarChangeListener {
    private final static String CLSS = "LidarFragment";
    private static final String LASER_SCAN_LAYER = "LASER_SCAN_LAYER";
    private String appName = null;
    private LaserListener laserListener = null;
    private TransformListener transformListener = null;
    private SBRosApplicationManager applicationManager;
    VisualizationView vizView = null;

    // Inflate the view. It shows Lidar output
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,Bundle savedInstanceState) {
        this.applicationManager = SBRosApplicationManager.getInstance();

        View view = inflater.inflate(R.layout.fragment_lidar, container, false);
        TextView label = view.findViewById(R.id.fragmentLidarText);
        label.setText(R.string.lidar_title);

        VerticalSeekBar seekBar = view.findViewById(R.id.verticalSeekbar);
        seekBar.setMax(100);
        seekBar.setOnSeekBarChangeListener(this);

        vizView = view.findViewById(R.id.fragmentLidarImage);
        List<Layer> layers = new ArrayList<>();
        layers.add(new LaserScanLayer(LASER_SCAN_LAYER));
        vizView.onCreate(layers);
        vizView.init();

        transformListener = new TransformListener();
        laserListener = new LaserListener();
        applicationManager.addListener(this);
        return view;
    }

    @Override
    public void onDestroyView() {
        Log.i(CLSS, "onDestroyView");
        applicationManager.removeListener(this);
        applicationShutdown(appName);
        vizView.onShutdown();
        super.onDestroyView();
    }
    // ========================================= SBApplicationStatusListener ============================
    // This may be called immediately on establishment of the listener.
    // We only react to specific applications.
    public void applicationStarted(String appName) {

        if(     !appName.equalsIgnoreCase(SBConstants.APPLICATION_FOLLOW) &&
                !appName.equalsIgnoreCase(SBConstants.APPLICATION_PARK) &&
                !appName.equalsIgnoreCase(SBConstants.APPLICATION_TELEOP))   return;

        Log.i(CLSS, String.format("applicationStarted: %s ...", appName));
        this.appName = appName;
        ConnectedNode node = applicationManager.getApplication().getConnectedNode();
        if (node != null) {
            transformListener.subscribe(node, "/tf_throttle");
            laserListener.subscribe(node,"/scan_throttle");
        }
        else {
            Log.i(CLSS, String.format("applicationStarted: %s has no connected node", appName));
        }

    }

    public void applicationShutdown(String appName) {
        if(     !appName.equalsIgnoreCase(SBConstants.APPLICATION_FOLLOW) &&
                !appName.equalsIgnoreCase(SBConstants.APPLICATION_PARK) &&
                !appName.equalsIgnoreCase(SBConstants.APPLICATION_TELEOP))   return;
        Log.i(CLSS, String.format("applicationShutdown"));
        transformListener.shutdown();
        laserListener.shutdown();
    }

    // ====================================== MessageListeners ================================================
    public class LaserListener extends AbstractMessageListener<LaserScan> {
        public LaserListener() { super(LaserScan._TYPE); }

        @Override
        public void onNewMessage(LaserScan scan) {
            Log.i(CLSS, String.format("Got a scan Message %s=%s",LaserScan._TYPE,scan.getClass().getCanonicalName()));
            Activity mainActivity = getActivity();
            if (mainActivity == null) {
                Log.i(CLSS, String.format("LaserListener: Main Activity no longer available"));
                return;
            }
            mainActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    vizView.onNewMessage(LASER_SCAN_LAYER,(LaserScan)scan);
                }
            });
        }
    }

    public class TransformListener extends AbstractMessageListener<tfMessage> {
        public TransformListener() {
            super(tfMessage._TYPE);
        }

        @Override
        public void onNewMessage(tfMessage tf) {
            Log.i(CLSS, String.format("Got a Transform Message"));
            Activity mainActivity = getActivity();
            if (mainActivity == null) {
                Log.i(CLSS, String.format("TransformListener: Main Activity no longer available"));
                return;
            }
            mainActivity.runOnUiThread(new Runnable() {
                // Message is destined for main visualizer only
                @Override
                public void run() {
                    vizView.onNewMessage(Layer.NO_LAYER,(tfMessage)tf);
                }
            });
        }
    }

    //===================================== OnSeekBarChangeListener ================================
    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {}

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {}

    /**
     * The seek bar goes from 0-100. Scale so that:
     *   0 = .25
     *   50=  1.0
     *   100= 4.
     * @param seekBar
     * @param progress
     * @param fromUser
     */
    @Override
    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
        Log.i(CLSS, String.format("progress changed %d",progress));
        if( vizView!=null ) {
            double setting = (double)(progress - 50)/25;
            double scale = Math.pow(2.0,setting);
            vizView.setScale(scale);
        }
    }
}
