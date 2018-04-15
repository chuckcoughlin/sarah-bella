/*
 * Copyright (C) 2014 Google Inc.
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

import com.google.common.base.Preconditions;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.android.view.visualization.Color;
import org.ros.android.view.visualization.Vertices;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.opengles.GL10;

import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;

/**
 * A layer that visualizes sensor_msgs/PointCloud2 messages in 2D.
 *
 * @author damonkohler@google.com (Damon Kohler)
 */
public class PointCloud2DLayer extends AbstractLayer implements TfLayer {
  private static final String CLSS = "PointCloud2DLayer";
  private static final Color FREE_SPACE_COLOR = Color.fromHexAndAlpha("377dfa", 0.1f);
  private static final Color OCCUPIED_SPACE_COLOR = Color.fromHexAndAlpha("377dfa", 0.3f);
  private static final float POINT_SIZE = 10.f;

  private GraphName frame;
  private FloatBuffer vertexFrontBuffer;
  private FloatBuffer vertexBackBuffer;


  public PointCloud2DLayer(String key) {
    super(key);
    this.frame = GraphName.of(CLSS);
  }


  public void onNewMessage(sensor_msgs.PointCloud2 message) {
    frame = GraphName.of(message.getHeader().getFrameId());
    updateVertexBuffer(message);
    visible= true;
  }

  @Override
  public void draw(VisualizationView view, GL10 gl) {
    if (vertexFrontBuffer != null) {
      synchronized (this) {
        Vertices.drawTriangleFan(gl, vertexFrontBuffer, FREE_SPACE_COLOR);
        // Drop the first point which is required for the triangle fan but is
        // not a range reading.
        FloatBuffer pointVertices = vertexFrontBuffer.duplicate();
        pointVertices.position(3);
        Vertices.drawPoints(gl, pointVertices, OCCUPIED_SPACE_COLOR, POINT_SIZE);
      }
    }
  }


  private void updateVertexBuffer(final PointCloud2 pointCloud) {
    // We expect an unordered, XYZ point cloud of 32-bit floats (i.e. the result of
    // pcl::toROSMsg()).
    // TODO(damonkohler): Make this more generic.
    Preconditions.checkArgument(pointCloud.getHeight() == 1);
    Preconditions.checkArgument(pointCloud.getIsDense());
    Preconditions.checkArgument(pointCloud.getFields().size() == 3);
    Preconditions.checkArgument(pointCloud.getFields().get(0).getDatatype() == PointField.FLOAT32);
    Preconditions.checkArgument(pointCloud.getFields().get(1).getDatatype() == PointField.FLOAT32);
    Preconditions.checkArgument(pointCloud.getFields().get(2).getDatatype() == PointField.FLOAT32);
    Preconditions.checkArgument(pointCloud.getPointStep() == 16);
    Preconditions.checkArgument(pointCloud.getData().order().equals(ByteOrder.LITTLE_ENDIAN));
    final int size = (pointCloud.getRowStep() / pointCloud.getPointStep() +
        1 /* triangle fan origin */) * 3 /* x, y, z */;
    if (vertexBackBuffer == null || vertexBackBuffer.capacity() < size) {
      vertexBackBuffer = Vertices.allocateBuffer(size);
    }
    vertexBackBuffer.clear();
    // We start with the origin of the triangle fan.
    vertexBackBuffer.put(0.f);
    vertexBackBuffer.put(0.f);
    vertexBackBuffer.put(0.f);

    final ChannelBuffer buffer = pointCloud.getData();
    while (buffer.readable()) {
      vertexBackBuffer.put(buffer.readFloat());
      vertexBackBuffer.put(buffer.readFloat());
      vertexBackBuffer.put(0.f);
      // Discard z data.
      buffer.readFloat();
      // Discard intensity.
      buffer.readFloat();
    }
    vertexBackBuffer.position(0);
    synchronized (this) {
      FloatBuffer tmp = vertexFrontBuffer;
      vertexFrontBuffer = vertexBackBuffer;
      vertexBackBuffer = tmp;
    }
  }

  @Override
  public GraphName getFrame() {
    return frame;
  }
}
