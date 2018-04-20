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

import android.support.v4.view.GestureDetectorCompat;
import android.util.Log;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;

import com.google.common.base.Preconditions;

import org.ros.android.view.visualization.RotateGestureDetector;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;

/**
 * Provides gesture control of the VisualizationView for translation
 * of all the layer output.
 */
public class LayerViewController {
    private static final String CLSS = "LayerViewController";
    private GestureDetectorCompat translateGestureDetector;

    /**
     * Called by the visualation view when it received a touch event.
     * The only event we handle is a long-press.
     *
     * @param view  the Visualization view
     * @param event the touch event
     */

    public boolean onTouchEvent(VisualizationView view, MotionEvent event) {
        if (translateGestureDetector == null) {
            return false;
        }
        return translateGestureDetector.onTouchEvent(event);
    }


    public void init(final VisualizationView view) {
        view.post(new Runnable() {
            @Override
            public void run() {
                translateGestureDetector =
                        new GestureDetectorCompat(view.getContext(), new GestureDetector.SimpleOnGestureListener() {
                            @Override
                            public boolean onDown(MotionEvent e) {
                                Log.i(CLSS, "onDown");
                                // This must return true in order for onScroll() to trigger.
                                return true;
                            }

                            ;

                        });
            }
        });
    }
}