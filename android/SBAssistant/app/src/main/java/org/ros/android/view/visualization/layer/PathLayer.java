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

import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.Color;
import geometry_msgs.PoseStamped;
import nav_msgs.Path;

import org.ros.internal.message.Message;
import org.ros.namespace.GraphName;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.opengles.GL10;

/**
 * Renders a nav_msgs/Path as a solid line.
 * 
 * @author moesenle@google.com (Lorenz Moesenlechner)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class PathLayer extends AbstractLayer implements TfLayer {
    private static final String CLSS = "PathLayer";
    private static final Color COLOR = Color.fromHexAndAlpha("03dfc9", 0.3f);
    private static final float LINE_WIDTH = 4.0f;

    private FloatBuffer vertexBuffer;
    private int numPoints;
    private GraphName frame;

    public PathLayer(String key) {
        super(key);
        this.frame = GraphName.of(CLSS);  // temporary
        numPoints = 0;
    }


    public void onNewMessage(nav_msgs.Path message) {
        frame = GraphName.of(message.getHeader().getFrameId());
        updateVertexBuffer(message);
        visible= true;
    }

    @Override
    public void draw(VisualizationView view, GL10 gl) {
        if (visible) {
            gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
            gl.glVertexPointer(3, GL10.GL_FLOAT, 0, vertexBuffer);
            COLOR.apply(gl);
            gl.glLineWidth(LINE_WIDTH);
            gl.glDrawArrays(GL10.GL_LINE_STRIP, 0, numPoints);
            gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);
        }
    }


    private void updateVertexBuffer(nav_msgs.Path path) {
        ByteBuffer goalVertexByteBuffer =
                ByteBuffer.allocateDirect(path.getPoses().size() * 3 * Float.SIZE);
        goalVertexByteBuffer.order(ByteOrder.nativeOrder());
        vertexBuffer = goalVertexByteBuffer.asFloatBuffer();
        int i = 0;
        if (path.getPoses().size() > 0) {
            for (PoseStamped pose : path.getPoses()) {
                vertexBuffer.put((float) pose.getPose().getPosition().getX());
                vertexBuffer.put((float) pose.getPose().getPosition().getY());
                vertexBuffer.put((float) pose.getPose().getPosition().getZ());
                i++;
            }
        }
        vertexBuffer.position(0);
        numPoints = i;
    }

    @Override
    public GraphName getFrame() { return this.frame; }
}
