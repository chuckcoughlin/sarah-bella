/*
 * Copyright (C) 2011 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.android.view.visualization;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.PixelFormat;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.util.Log;
import android.view.MotionEvent;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import org.ros.android.view.visualization.layer.LayerViewController;
import org.ros.android.view.visualization.layer.Layer;
import org.ros.android.view.visualization.layer.RobotViewController;
import org.ros.internal.message.Message;

import org.ros.namespace.GraphName;
import org.ros.rosjava_geometry.FrameTransformTree;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import tf.tfMessage;

/**
 * @author damonkohler@google.com (Damon Kohler)
 * @author moesenle@google.com (Lorenz Moesenlechner)
 */
public class VisualizationView extends GLSurfaceView  {
    private static final String CLSS = "VisualizationView";
    private static final String INITIAL_FRAME = "root";
    private static final boolean DEBUG = false;

    private final FrameTransformTree frameTransformTree = new FrameTransformTree();
    private final XYOrthographicCamera camera = new XYOrthographicCamera(frameTransformTree);

    private List<Layer> layers;
    private Map<String,Layer> layerMap ;
    private XYOrthographicRenderer renderer;
    private LayerViewController layerController;
    private RobotViewController robotController;

    public VisualizationView(Context context) {
        super(context);
    }
    public VisualizationView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    /**
     * Create a lookup between layers and message class.
     * @param layers
     */
    public void onCreate(List<Layer> layers) {
        this.layerController = new LayerViewController();
        this.robotController = new RobotViewController(this);
        this.layers = layers;
        layerMap = new HashMap<>();
        for(Layer layer:layers) {
            layerMap.put(layer.getKey(),layer);
        }

        setDebugFlags(DEBUG_CHECK_GL_ERROR);
        if (DEBUG) {
            // Turn on OpenGL logging.
            setDebugFlags(getDebugFlags() | DEBUG_LOG_GL_CALLS);
        }
        setEGLConfigChooser(8, 8, 8, 8, 0, 0);
        getHolder().setFormat(PixelFormat.TRANSLUCENT);
        renderer = new XYOrthographicRenderer(this);
        setRenderer(renderer);
        setRenderMode(RENDERMODE_WHEN_DIRTY);
    }

    /**
     *
     */
    public void init() {
        Preconditions.checkNotNull(layers);
        //robotController.draw(this,gl);
        for (Layer layer : layers) {
            layer.init(this);
        }
    }

    @Override
    public void draw(Canvas canvas) {
        super.draw(canvas);
        Log.i(CLSS,"Drawing view !!!!!!!!!!!!!!");
    }

    /**
     * The long-press is the only event that we've been able to observe.
     * (which is fine). We use this to signal a translation of our robot
     * marker to the place pressed on the screen.
     * @param event
     * @return
     */
    @Override
    public boolean onTouchEvent(MotionEvent event) {
        Log.i(CLSS,String.format("onTouchEvent %d",event.getAction()));
        layerController.onTouchEvent(this,event);
        robotController.onTouchEvent(this,event);
        return super.onTouchEvent(event);
    }

    public XYOrthographicRenderer getRenderer() {
        return renderer;
    }
    public XYOrthographicCamera getCamera() {
        return camera;
    }
    public RobotViewController getRobotController() { return robotController; }
    public FrameTransformTree getFrameTransformTree() {
        return frameTransformTree;
    }
    public List<Layer> getLayers() {
        return Collections.unmodifiableList(layers);
    }

    /**
     * Translate each layer to new x,y origin.
     */
    public void translate(int x,int y) {
        for (Layer layer : layers) {
            layer.translate(this,x,y);
        }
        requestRender();
    }
    /**
     * Scale the drawing, then re-render.
     */
    public void setScale(double scale) {
        for (Layer layer : layers) {
            layer.setScale(scale);
        }
        requestRender();
    }
    /**
     * Use the key to lookup the proper layer. Forward the message to it.
     * @param message
     */
    public void onNewMessage(String key,Message message) {
        if(message instanceof tf.tfMessage ) {
            synchronized (this) {
                tfMessage tmessage = (tfMessage)message;
                for (geometry_msgs.TransformStamped transform : tmessage.getTransforms()) {
                    frameTransformTree.update(transform);
                }
            }
        }
        Layer layer = layerMap.get(key);
        if( layer!=null ) {
            //Log.i(CLSS,String.format("Sending message to layer (%s). ",layer.getClass().getCanonicalName()));
            layer.onNewMessage(message);
        }
        requestRender();  // Update the layers
    }

    public void onShutdown() {
        for (Layer layer : layers) {
            layer.setVisible(false);
        }
    }
}