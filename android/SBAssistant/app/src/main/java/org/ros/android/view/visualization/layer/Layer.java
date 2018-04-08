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
 *
 * We've modified to remove references to NodeMain and executors. The responsibility for
 * subscriptions is handled outside the view and its layers. Each layer is responsible for
 * responding (or not) to the incoming messages.
 */

package org.ros.android.view.visualization.layer;

import android.opengl.GLSurfaceView;
import android.view.MotionEvent;

import org.ros.android.view.visualization.OpenGlDrawable;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.internal.message.Message;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * Interface for a drawable layer on a VisualizationView.
 *
 * @author moesenle@google.com (Lorenz Moesenlechner)
 */
public interface Layer extends OpenGlDrawable {

    /**
     */
    public void init(VisualizationView view);

    /**
     * @return the message type handled by this layer
     */
    public String getMessageType();

    /**
     * @return true if the layer is currently displayed and
     * handling messages
     */
    public boolean isVisible();

    /**
     * Turn the layer on or off. When off it should become transparent.
     * @param flag
     */
    public void setVisible(boolean flag);

    // =================================== Message Handlers ========================================
    /*
     * Each layer is expected to override this method for its specific message type.
     */
    public void onNewMessage(Message message);


    // ======================================== ?? ================================================
    /**
     * Event handler for touch events.
     *
     * @param view the view generating the event
     * @param event the touch event
     * @return true if the event has been handled
     */
    boolean onTouchEvent(VisualizationView view, MotionEvent event);


}
