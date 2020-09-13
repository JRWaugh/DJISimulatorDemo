package com.dji.videostreamdecodingsample;

import android.graphics.Rect;
import android.graphics.YuvImage;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import sensor_msgs.CompressedImage;

public class CompressedImagePublisher {
    private final ConnectedNode connectedNode;
    private final Publisher<CompressedImage> imagePublisher;
    private final Publisher<sensor_msgs.CameraInfo> cameraInfoPublisher;
    private final ChannelBufferOutputStream stream;

    public CompressedImagePublisher(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        imagePublisher = connectedNode.newPublisher("camera/image/compressed", sensor_msgs.CompressedImage._TYPE);
        cameraInfoPublisher = connectedNode.newPublisher("camera/info", sensor_msgs.CameraInfo._TYPE);
        stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
    }

    public void onNewImage(YuvImage yuv) {
        Time currentTime = connectedNode.getCurrentTime();
        String frameId = "camera";

        sensor_msgs.CompressedImage image = imagePublisher.newMessage();
        image.getHeader().setStamp(currentTime);
        image.getHeader().setFrameId(frameId);
        image.setFormat("jpeg");
        yuv.compressToJpeg(new Rect(0, 0, yuv.getWidth(), yuv.getHeight()), 80, stream);
        image.setData(stream.buffer().copy());
        stream.buffer().clear();
        imagePublisher.publish(image);

        sensor_msgs.CameraInfo cameraInfo = cameraInfoPublisher.newMessage();
        cameraInfo.getHeader().setStamp(currentTime);
        cameraInfo.getHeader().setFrameId(frameId);
        cameraInfo.setWidth(1280);
        cameraInfo.setHeight(720);
        cameraInfoPublisher.publish(cameraInfo);
    }
}
