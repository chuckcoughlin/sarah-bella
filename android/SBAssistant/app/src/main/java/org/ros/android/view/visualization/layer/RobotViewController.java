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

package org.ros.android.view.visualization.layer;

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
    private Shape shape;
    private Transform pose;
    private boolean visible;
    private GestureDetector gestureDetector;

    public RobotViewController(VisualizationView view) {
        this.shape = new PixelSpacePoseShape();
        this.visible = false;
        gestureDetector =
                new GestureDetector(view.getContext(), new GestureDetector.SimpleOnGestureListener() {
                    @Override
                    public void onLongPress(MotionEvent e) {
                        Log.i(CLSS,"GestureDetector: LongPress: translating");
                        pose = Transform.translation(view.getCamera().toCameraFrame((int) e.getX(),
                                        (int) e.getY()));
                        shape.setTransform(pose);
                        visible = true;
                    }
                });
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
        if (visible) {
            Preconditions.checkNotNull(pose);

            if (event.getAction() == MotionEvent.ACTION_MOVE) {
                Vector3 poseVector = pose.apply(Vector3.zero());
                Vector3 pointerVector =
                        view.getCamera().toCameraFrame((int) event.getX(), (int) event.getY());
                double angle =
                        angle(pointerVector.getX(), pointerVector.getY(), poseVector.getX(), poseVector.getY());
                pose = Transform.translation(poseVector).multiply(Transform.zRotation(angle));
                shape.setTransform(pose);
                return true;
            }
        }
        gestureDetector.onTouchEvent(event);
        view.requestRender();  // Redraw
        return false;
    }

}