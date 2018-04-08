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

import com.google.common.base.Preconditions;

import org.ros.android.view.visualization.VisualizationView;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.android.view.visualization.TextureBitmap;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.rosjava_geometry.Transform;

import javax.microedition.khronos.opengles.GL10;

/**
 * NOTE: Because we have assumed a 1:1 relationship between message types and
 * layers, only one of OccupancyGridLayer and CompressedOccupancyGridLayer
 * can be active at a time.
 *
 * @author damonkohler@google.com (Damon Kohler)
 * @author moesenle@google.com (Lorenz Moesenlechner)
 */
public class CompressedOccupancyGridLayer extends AbstractLayer implements TfLayer {
    private static final String CLSS = "CompressedOccupancyGridLayer";
    private static final String MESSAGE_TYPE = "nav_msgs/OccupancyGrid";
    private static final int COLOR_OCCUPIED = 0xdfffffff;     // Color of occupied cells in the map.
    private static final int COLOR_FREE = 0xff8d8d8d;         // Color of free cells in the map.
    private static final int COLOR_UNKNOWN = 0xff000000;      // Color of unknown cells in the map.



    private final TextureBitmap textureBitmap;

    private GraphName frame;
    public CompressedOccupancyGridLayer() {
        super();
        this.frame = GraphName.of(CLSS);  // temporary
        textureBitmap = new TextureBitmap();
    }

    @Override
    public String getMessageType() { return MESSAGE_TYPE; }

    public void onNewMessage(nav_msgs.OccupancyGrid message) {
        frame = GraphName.of(message.getHeader().getFrameId());
        update(message);
        visible= true;
    }

    @Override
    public void draw(VisualizationView view, GL10 gl) {
        if (visible) {
            textureBitmap.draw(view, gl);
        }
    }

    private void update(nav_msgs.OccupancyGrid message) {
        ChannelBuffer buffer = message.getData();
        Bitmap bitmap =
                BitmapFactory.decodeByteArray(buffer.array(), buffer.arrayOffset(), buffer.readableBytes());
        int stride = bitmap.getWidth();
        int height = bitmap.getHeight();
        Preconditions.checkArgument(stride <= 1024);
        Preconditions.checkArgument(height <= 1024);
        int[] pixels = new int[stride * height];
        bitmap.getPixels(pixels, 0, stride, 0, 0, stride, height);
        for (int i = 0; i < pixels.length; i++) {
            // Pixels are ARGB packed ints.
            if (pixels[i] == 0xffffffff) {
                pixels[i] = COLOR_UNKNOWN;
            } else if (pixels[i] == 0xff000000) {
                pixels[i] = COLOR_FREE;
            } else {
                pixels[i] = COLOR_OCCUPIED;
            }
        }
        float resolution = message.getInfo().getResolution();
        Transform origin = Transform.fromPoseMessage(message.getInfo().getOrigin());
        textureBitmap.updateFromPixelArray(pixels, stride, resolution, origin, COLOR_UNKNOWN);
        frame = GraphName.of(message.getHeader().getFrameId());
        visible = true;
    }
    @Override
    public GraphName getFrame() {
        return frame;
    }
}
