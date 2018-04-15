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
    protected boolean visible = false;
    protected VisualizationView view = null;

    /**
     * Constructor
     * @param id identifier set by the user on layer creation
     */
    public AbstractLayer(String id) {
        this.key = id;
        this.visible = false;
    }

    @Override
    public String getKey() { return this.key; }

    @Override
    public abstract void draw(VisualizationView view, GL10 gl);

    @Override
    public void init(VisualizationView viz) {this.view = viz;}

    @Override
    public boolean isVisible() { return visible; }

    @Override
    public void setVisible(boolean flag) { this.visible = flag; }

    // =================================== Message Handler ========================================
    /*
     * Each layer is expected to implement this method for its specific message types.
     * If we get here then some layer was called with the wrong message type.
     */
    @Override
    public void onNewMessage(Message message) {
        Log.i(CLSS,String.format("WARNING: %s.onNewMessage called with unhandled class %s",getClass().getCanonicalName(),
                message.getClass().getCanonicalName()));
    }

    /**
     * Only the camera layer responds to this. For the remaining layers, do nothing.
     * @param view the view generating the event
     * @param event the touch event
     * @return success or failure
     */
    @Override
    public boolean onTouchEvent(VisualizationView view, MotionEvent event) {
        return false;
    }

}
