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
import android.widget.TextView;

import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.layer.LaserScanLayer;
import org.ros.android.view.visualization.layer.Layer;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import chuckcoughlin.sb.assistant.R;
import chuckcoughlin.sb.assistant.common.AbstractMessageListener;
import chuckcoughlin.sb.assistant.common.SBConstants;
import chuckcoughlin.sb.assistant.ros.SBApplicationStatusListener;
import chuckcoughlin.sb.assistant.ros.SBRosApplicationManager;
import chuckcoughlin.sb.assistant.ros.SBRosManager;
import gpio_msgs.GPIOSet;
import sensor_msgs.LaserScan;
import tf.tfMessage;


/**
 * This fragment handles robot control during its SLAM
 * mapping sequence.
 */

public class LidarFragment extends BasicAssistantFragment implements SBApplicationStatusListener {
    private final static String CLSS = "LidarFragment";
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

        vizView = view.findViewById(R.id.fragmentLidarImage);
        List<Layer> layers = new ArrayList<>();
        layers.add(new LaserScanLayer());
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
        applicationShutdown();
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

        ConnectedNode node = applicationManager.getApplication().getConnectedNode();
        if (node != null) {
            transformListener.subscribe(node, "/tf");
            laserListener.subscribe(node,"/scan");
        }
        else {
            Log.i(CLSS, String.format("applicationStarted: %s has no connected node", appName));
        }

    }

    public void applicationShutdown() {
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
                    vizView.onNewMessage(scan);
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
                @Override
                public void run() {
                    vizView.onNewMessage(tf);
                }
            });
        }
    }
}
