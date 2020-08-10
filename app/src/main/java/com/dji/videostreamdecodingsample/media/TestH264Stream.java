package com.dji.videostreamdecodingsample.media;

import android.content.Context;
import android.media.MediaCodec;
import android.media.MediaFormat;

import com.pedro.encoder.video.FormatVideoEncoder;
import com.pedro.encoder.video.GetVideoData;
import com.pedro.encoder.video.VideoEncoder;

import net.ossrs.rtmp.ConnectCheckerRtmp;
import net.ossrs.rtmp.SrsFlvMuxer;

import java.nio.ByteBuffer;

public class TestH264Stream implements GetVideoData, ConnectCheckerRtmp {

    //private VideoDecoder videoDecoder;
    private DJIVideoStreamDecoder decoder;
    private VideoEncoder videoEncoder;
    private SrsFlvMuxer srsFlvMuxer;
    private int width, height;

    public TestH264Stream() {
        decoder = DJIVideoStreamDecoder.getInstance();
        videoEncoder = new VideoEncoder(this);
        srsFlvMuxer = new SrsFlvMuxer(this);
    }

    public void prepare(int width, int height) {
        this.width = width;
        this.height = height;
        videoEncoder.prepareVideoEncoder(width, height, 30, 1200 * 1024, 0, false, 2, FormatVideoEncoder.SURFACE);
    }

    public void start(Context context, String url) {
        videoEncoder.start();
        decoder.init(context, videoEncoder.getInputSurface());
        srsFlvMuxer.setVideoResolution(width, height);
        srsFlvMuxer.start(url);
    }

    public void stop() {
        decoder.stop();
        videoEncoder.stop();
        srsFlvMuxer.stop();
    }

    @Override
    public void onSpsPps(ByteBuffer sps, ByteBuffer pps) {
        srsFlvMuxer.setSpsPPs(sps, pps);
    }

    @Override
    public void onSpsPpsVps(ByteBuffer sps, ByteBuffer pps, ByteBuffer vps) {
        srsFlvMuxer.setSpsPPs(sps, pps);
    }

    @Override
    public void getVideoData(ByteBuffer h264Buffer, MediaCodec.BufferInfo info) {
        srsFlvMuxer.sendVideo(h264Buffer, info);
    }

    @Override
    public void onVideoFormat(MediaFormat mediaFormat) {

    }

    @Override
    public void onConnectionSuccessRtmp() { }

    @Override
    public void onConnectionFailedRtmp(String reason) { }

    @Override
    public void onNewBitrateRtmp(long bitrate) { }

    @Override
    public void onDisconnectRtmp() { }

    @Override
    public void onAuthErrorRtmp() { }

    @Override
    public void onAuthSuccessRtmp() { }
}
