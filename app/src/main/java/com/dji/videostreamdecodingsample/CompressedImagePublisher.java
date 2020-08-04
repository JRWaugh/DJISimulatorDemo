package com.dji.videostreamdecodingsample;

import android.graphics.Rect;
import android.graphics.YuvImage;
import android.util.Log;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.NameResolver;
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
        NameResolver resolver = connectedNode.getResolver().newChild("camera");
        imagePublisher = connectedNode.newPublisher(resolver.resolve("image/compressed"), sensor_msgs.CompressedImage._TYPE);
        cameraInfoPublisher = connectedNode.newPublisher(resolver.resolve("info"), sensor_msgs.CameraInfo._TYPE);
        stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
    }

    public void onNewImage(YuvImage yuv) {
        Log.d("TAG", "Should be streamin!");
        Time currentTime = connectedNode.getCurrentTime();
        String frameId = "camera";

        sensor_msgs.CompressedImage image = imagePublisher.newMessage();
        image.setFormat("jpeg");
        image.getHeader().setStamp(currentTime);
        image.getHeader().setFrameId(frameId);
        yuv.compressToJpeg(new Rect(0, 0, 1280, 720), 80, stream);
        //if (bitmap.compress(Bitmap.CompressFormat.JPEG, 80, stream))
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
