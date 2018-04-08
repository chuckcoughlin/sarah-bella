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
import org.ros.android.view.visualization.shape.GoalShape;
import org.ros.android.view.visualization.shape.Shape;
import org.ros.internal.message.Message;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.rosjava_geometry.FrameTransform;
import org.ros.rosjava_geometry.Transform;

import javax.microedition.khronos.opengles.GL10;

import geometry_msgs.PoseStamped;
import nav_msgs.Path;

/**
 * @author moesenle@google.com (Lorenz Moesenlechner)
 */
public class PoseLayer extends AbstractLayer implements TfLayer {
  private static final String CLSS = "PoseLayer";
  private static final String MESSAGE_TYPE = "geometry_msgs/PoseStamped";
  private final GraphName frame;

  private Shape shape;

  public PoseLayer() {
    super();
    this.frame = GraphName.of(CLSS);
  }
  @Override
  public String getMessageType() { return MESSAGE_TYPE; }

  public void onNewMessage(geometry_msgs.PoseStamped message) {
      GraphName source = GraphName.of(message.getHeader().getFrameId());
      FrameTransform frameTransform = view.getFrameTransformTree().transform(source, frame);
      if (frameTransform != null) {
          Transform poseTransform = Transform.fromPoseMessage(message.getPose());
          shape.setTransform(frameTransform.getTransform().multiply(poseTransform));
          visible = true;
      }
    visible= true;
  }
  @Override
  public void draw(VisualizationView view, GL10 gl) {
    if (visible) {
      shape.draw(view, gl);
    }
  }

  @Override
  public GraphName getFrame() {
    return frame;
  }
}