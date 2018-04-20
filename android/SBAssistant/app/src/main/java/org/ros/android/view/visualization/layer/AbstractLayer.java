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

package org.ros.android.view.visualization.layer;

import android.util.Log;
import android.view.MotionEvent;

import org.ros.android.view.visualization.VisualizationView;
import org.ros.internal.message.Message;
import org.ros.namespace.GraphName;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * Base class for visualization layers.
 *
 * @author moesenle@google.com (Lorenz Moesenlechner)
 */
public abstract class AbstractLayer implements Layer {
    private static final String CLSS = "AbstractLayer";
    protected final String key;
    protected double scale    = 1.0;
    protected boolean visible = false;
    protected VisualizationView view = null;
    protected GraphName frame = null;

    /**
     * Constructor
     * @param id identifier set by the user on layer creation
     */
    public AbstractLayer(String id) {
        this.key = id;
        this.visible = false;
        this.frame = null;
    }

    @Override
    public String getKey() { return this.key; }

    @Override
    public abstract void draw(VisualizationView view, GL10 gl);

    @Override
    public void init(VisualizationView viz) {this.view = viz;}

    @Override
    public boolean isVisible() { return visible; }

    protected double getScale() { return this.scale; }
    @Override
    public void setScale(double s) { this.scale = s; }

    @Override
    public void setVisible(boolean flag) { this.visible = flag; }

    // =================================== Message Handler ========================================
    /*
     * Each layer is expected to implement this method for its specific message types.
     * If we get here then some layer was called with the wrong message type.
     */
    @Override
    public abstract void onNewMessage(Message message);

    /**
     * For layers that are positioned by using Tf.
     * The graph name is simply a string.
     * @return the {@link Layer}'s reference frame
     */
    public GraphName getFrame() { return this.frame; }

    /**
     * Translate coordinates with respect to new origin. By default
     * we do nothing.
     */
    public void translate(VisualizationView vizView,int x,int y) {

    }

}
