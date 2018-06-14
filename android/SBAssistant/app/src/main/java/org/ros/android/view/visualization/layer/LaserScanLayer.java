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

import org.ros.android.view.visualization.Color;
import org.ros.android.view.visualization.Vertices;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.internal.message.Message;
import org.ros.namespace.GraphName;
import org.ros.rosjava_geometry.Transform;

import java.nio.FloatBuffer;
import javax.microedition.khronos.opengles.GL10;
import sensor_msgs.LaserScan;

/**
 * A Layer that visualizes sensor_msgs/LaserScan messages. Rotate 90 deg
 * clockwise so that the top of the screen corresponds to straight ahead.
 *
 * @author munjaldesai@google.com (Munjal Desai)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class LaserScanLayer extends AbstractLayer  {
    private static final String CLSS = "LaserScanLayer";
    private static final Color BORDER_COLOR = Color.fromHexAndAlpha("303f9f", 0.9f);
    private static final Color FREE_SPACE_COLOR = Color.fromHexAndAlpha("377dfa", 0.1f);
    private static final Color OCCUPIED_SPACE_COLOR = Color.fromHexAndAlpha("377dfa", 0.3f);
    private static final float LASER_SCAN_LINE_WIDTH = 1.f;   // The only supported width?
    private static final float LASER_SCAN_POINT_SIZE = 10.f;
    //private static final int LASER_SCAN_STRIDE = 15;    // Show every 15th data point
    private static final int LASER_SCAN_STRIDE = 1;     // Show all data points

    private GraphName frame;
    private LaserScan message = null;    // Save prior message
    private FloatBuffer vertexFrontBuffer;
    private FloatBuffer vertexBackBuffer;

    public LaserScanLayer(String key) {
        super(key);
        this.frame = GraphName.of(CLSS);  // temporary
    }


    /**
     * We count on the Visualization view to properly disseminate by class
     * @param m a LaserScan message
     */
    @Override
    public void onNewMessage(Message m) {
        Log.i(CLSS, "LaserScan got new message ------------------------------------");
        message = (LaserScan) m;
        frame = GraphName.of(message.getHeader().getFrameId());
        updateVertexBuffer(message, LASER_SCAN_STRIDE);
        visible = true;
    }

    @Override
    public void draw(VisualizationView view, GL10 gl) {
        //Log.i(CLSS, "LaserScan layer draw ------------------------------------");
        if (vertexFrontBuffer != null) {
            synchronized (this) {
                Vertices.drawTriangleFan(gl, vertexFrontBuffer, FREE_SPACE_COLOR);
                // Drop the first point which is required for the triangle fan but is
                // not a range reading.
                FloatBuffer pointVertices = vertexFrontBuffer.duplicate();
                pointVertices.position(3);
                Vertices.drawPoints(gl, pointVertices, OCCUPIED_SPACE_COLOR, LASER_SCAN_POINT_SIZE);
                pointVertices.position(3);
                Vertices.drawLines(gl, pointVertices, BORDER_COLOR, LASER_SCAN_LINE_WIDTH);
            }
        }
    }

    @Override
    public void setScale(double s) {
        super.setScale(s);
        updateVertexBuffer(message,LASER_SCAN_STRIDE);
    }

    @Override
    public void setTransform(Transform tr) {
        super.setTransform(tr);
        updateVertexBuffer(message,LASER_SCAN_STRIDE);
    }

    private void updateVertexBuffer(LaserScan laserScan, int stride) {
        if(laserScan==null) return;

        int vertexCount = 0;
        float[] ranges = laserScan.getRanges();
        int size = ((ranges.length / stride) + 2) * 3;
        if (vertexBackBuffer == null || vertexBackBuffer.capacity() < size) {
            vertexBackBuffer = Vertices.allocateBuffer(size);
        }
        vertexBackBuffer.clear();
        // We start with the origin of the triangle fan.
        float originx = 0.0f;
        float originy = 0.0f;
        if( pose!=null ) {
            originx = (float)pose.getTranslation().getX();
            originy = (float)pose.getTranslation().getY();
        }
        vertexBackBuffer.put(originx);
        vertexBackBuffer.put(originy);
        vertexBackBuffer.put(0);
        vertexCount++;
        float minimumRange = laserScan.getRangeMin();
        float maximumRange = laserScan.getRangeMax();
        // The raw angular data has 0 deg to the left.
        // Modify so that 0 deg is straight ahead.
        float angle = (float)(laserScan.getAngleMin() - Math.PI/2.);
        float angleIncrement = laserScan.getAngleIncrement();
        // Calculate the coordinates of the laser range values.
        for (int i = 0; i < ranges.length; i += stride) {
            float range = ranges[i];
            // Ignore ranges that are outside the defined range. We are not overly
            // concerned about the accuracy of the visualization and this is makes it
            // look a lot nicer.
            if (minimumRange < range && range < maximumRange) {
                // x, y, z
                vertexBackBuffer.put(originx + (float)(range * scale * Math.cos(angle)));
                vertexBackBuffer.put(originy + (float)(range * scale * Math.sin(angle)));
                vertexBackBuffer.put(0);
                vertexCount++;
            }
            angle += angleIncrement * stride;
        }
        vertexBackBuffer.position(0);
        vertexBackBuffer.limit(vertexCount * 3);
        synchronized (this) {
            FloatBuffer tmp = vertexFrontBuffer;
            vertexFrontBuffer = vertexBackBuffer;
            vertexBackBuffer = tmp;
        }
        Log.i(CLSS, String.format("%d vertices", vertexCount));
    }

    @Override
    public GraphName getFrame() {
        return frame;
    }
}
