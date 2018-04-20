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
 * Translate our origin on the screen in response to a LongPress.
 */

package org.ros.android.view.visualization;

import android.util.Log;
import android.view.GestureDetector;
import android.view.MotionEvent;

import com.google.common.base.Preconditions;

import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.shape.PixelSpacePoseShape;
import org.ros.android.view.visualization.shape.Shape;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import javax.microedition.khronos.opengles.GL10;

/**
 * @author moesenle@google.com (Lorenz Moesenlechner)
 */
public class RobotViewController {
    private static final String CLSS = "RobotViewController";
    private final VisualizationView view;
    private boolean initialized = false;
    private Shape shape;
    private Transform pose = null;
    private boolean visible;

    public RobotViewController(VisualizationView view) {
        Log.i(CLSS,"setting inital transform");
        this.view = view;
        this.visible = false;
    }

    // Place the first "pose" in the center of the screen.
    private void init() {
        int initialX = view.getWidth()/2;
        int initialY = view.getHeight()/2;
        this.shape = new PixelSpacePoseShape();
        this.pose = Transform.translation(view.getCamera().toCameraFrame(initialX,initialY));
        shape.setTransform(pose);
        visible = true;
        initialized = true;
    }

    public void draw(VisualizationView view, GL10 gl) {
        if (shape != null) {
            shape.draw(view, gl);
        }
    }

    private double angle(double x1, double y1, double x2, double y2) {
        double deltaX = x1 - x2;
        double deltaY = y1 - y2;
        return Math.atan2(deltaY, deltaX);
    }

    /**
     *
     * @param view
     * @param event
     * @return
     */
    public boolean onTouchEvent(VisualizationView view, MotionEvent event) {
        boolean result = false;
        if (!initialized) init();
        Preconditions.checkNotNull(pose);
        if (event.getAction() == MotionEvent.ACTION_DOWN) {
            Log.i(CLSS, String.format("ACTION_DOWN at %d,%d", (int) event.getX(), (int) event.getY()));
            pose = Transform.translation(view.getCamera().toCameraFrame((int) event.getX(), (int) event.getY()));
            shape.setTransform(pose);
            result = true;
        }
        // This is never invoked, but the code acts to aim the pointer at the touch point
        else if (event.getAction() == MotionEvent.ACTION_MOVE) {
            Log.i(CLSS, "ACTION_MOVE");
            Vector3 poseVector = pose.apply(Vector3.zero());
            Vector3 pointerVector =
                    view.getCamera().toCameraFrame((int) event.getX(), (int) event.getY());
            double angle =
                    angle(pointerVector.getX(), pointerVector.getY(), poseVector.getX(), poseVector.getY());
            pose = Transform.translation(poseVector).multiply(Transform.zRotation(angle));
            shape.setTransform(pose);
            result = true;
        }
        view.requestRender();  // Redraw
        return result;
    }

}