package com.dji.videostreamdecodingsample;

import android.content.Intent;
import android.graphics.ImageFormat;
import android.graphics.YuvImage;
import android.media.MediaFormat;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;

import com.dji.videostreamdecodingsample.media.DJIVideoStreamDecoder;
import com.dji.videostreamdecodingsample.media.NativeHelper;
import com.pedro.rtplibrary.rtmp.RtmpCamera1;

import net.ossrs.rtmp.ConnectCheckerRtmp;

import org.ros.android.RosActivity;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Timer;
import java.util.TimerTask;


import net.ossrs.rtmp.ConnectCheckerRtmp;

import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.common.flightcontroller.ConnectionFailSafeBehavior;
import dji.common.flightcontroller.ObstacleDetectionSector;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.gimbal.GimbalMode;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.flightcontroller.FlightAssistant;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;
import dji.sdk.sdkmanager.LiveStreamManager;
import dji.sdk.useraccount.UserAccountManager;

import static dji.common.flightcontroller.virtualstick.Limits.YAW_CONTROL_MAX_ANGLE;
import static dji.common.flightcontroller.virtualstick.Limits.YAW_CONTROL_MIN_ANGLE;

public class CameraActivity extends RosActivity implements NodeMain {
    private static final String TAG = CameraActivity.class.getName();
    private static final String STREAM_URL = "rtmp://84.248.73.112:1935/hls/test";
    final static float LOWER_IDEAL_DISTANCE = 0.87f;
    final static float UPPER_IDEAL_DISTANCE = 1.22f;
    final static float WALL_LOST_DISTANCE = 2.2f;
    final static int WIDTH = 1280;
    final static int HEIGHT = 720;
    final static int YUV_DATA_LENGTH = WIDTH * HEIGHT * 3 / 2;

    enum State {
        NoWall, // No obstacle or wall within 2 metres. Move forward at 0.2m/s.
        WallAhead,
        WallStart,
        WallResume,
        StartNewVector,
        WallFollow,
        WallComplete,
        WallDisjointed, // Wall ended before reaching safe point, so it is disjointed
    }

    public static class Bearing {
        public Bearing(final float currentBearing) {
            mNorthBearing = currentBearing;

            mWestBearing = mNorthBearing - 90.0f;
            if (mWestBearing < YAW_CONTROL_MIN_ANGLE)
                mWestBearing = YAW_CONTROL_MAX_ANGLE + (mWestBearing - YAW_CONTROL_MIN_ANGLE);

            mEastBearing = mNorthBearing + 90.0f;
            if (mEastBearing > YAW_CONTROL_MAX_ANGLE)
                mEastBearing = YAW_CONTROL_MIN_ANGLE + (mEastBearing - YAW_CONTROL_MAX_ANGLE);
        }

        public float getNorthBearing() {
            return mNorthBearing;
        }

        public float getEastBearing() {
            return mEastBearing;
        }

        public float getWestBearing() {
            return mWestBearing;
        }

        private float mEastBearing;
        private final float mNorthBearing;
        private float mWestBearing;
    }

    enum TargetRelativeBearing {
        NORTH, EAST, SOUTH, WEST
    }

    private FlightController mFlightController;
    private FlightAssistant mFlightAssistant;
    private Gimbal mGimbal;
    private FlightControlData mFlightControlData;

    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener;
    private Timer mSendVirtualStickDataTimer;
    private SendVirtualStickDataTask mSendVirtualStickDataTask;

    private TargetRelativeBearing mTargetBearing = TargetRelativeBearing.WEST;
    private Bearing mBearing = null;
    private State mState = State.NoWall;
    private byte[] mYUV = null;
    private HandlerThread handlerThread;
    private CompressedImagePublisher compressedImagePublisher;
    private SurfaceView mSurfaceView;
    private RtmpCamera1 mRTMPCamera;


    /*  *  *  ACTIVITY METHOD OVERRIDES *  *  */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.e(TAG, "onCreate");
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);

        handlerThread = new HandlerThread("ImageHandler");
        handlerThread.start();
        initUI();
        mYUV = new byte[YUV_DATA_LENGTH];
        Arrays.fill(mYUV, (byte) 128); // We don't care about the chrominance (U and V), so we can set all that data to no-chrominance and only read Y data

        /*
        DJIVideoStreamDecoder.getInstance().init(this, null);
        DJIVideoStreamDecoder.getInstance().setYuvDataListener((MediaFormat format, final ByteBuffer yuvFrame, int dataSize, final int width, final int height) -> {
            if (yuvFrame != null) {
                yuvFrame.get(mYUV, 0, width * height);
                YuvImage yuv = new YuvImage(mYUV, ImageFormat.NV21, width, height, null);
                Handler handler = new Handler(handlerThread.getLooper());
                handler.post(() -> {
                    if (compressedImagePublisher != null)
                        compressedImagePublisher.onNewImage(yuv);
                });
            }
        });

         */

        mReceivedVideoDataListener = (bytes, i) -> DJIVideoStreamDecoder.getInstance().parse(bytes, i);
        VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(mReceivedVideoDataListener);
        mFlightControlData = new FlightControlData(0.0f, 0.0f, 0.0f, 0.0f);
    }

    @Override
    public void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();

        DJIVideoStreamDecoder.getInstance().resume();
        VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(mReceivedVideoDataListener);

        AsyncTask.execute(() -> DJISDKManager.getInstance().registerApp(this, new DJISDKManager.SDKManagerCallback() {
            @Override
            public void onRegister(DJIError djiError) {
                Log.d(TAG, djiError.getDescription());

                if (djiError == DJISDKError.REGISTRATION_SUCCESS) {
                    UserAccountManager.getInstance().logIntoDJIUserAccount(getApplicationContext(), null);
                    DJISDKManager.getInstance().startConnectionToProduct();
                }
            }

            @Override
            public void onProductConnect(BaseProduct baseProduct) {
                showToast("Product Connected");

                setFlightController(((Aircraft) baseProduct).getFlightController());
                setFlightAssistant(((Aircraft) baseProduct).getFlightController().getFlightAssistant());
                setGimbal(baseProduct.getGimbal());
                initVirtualStickData();
            }

            @Override
            public void onProductChanged(BaseProduct baseProduct) {

            }

            @Override
            public void onProductDisconnect() {
                Log.d(TAG, "onProductDisconnect");

                setFlightController(null);
                setFlightAssistant(null);
                setGimbal(null);
                unInitVirtualStickData();

                startActivity(new Intent(getApplicationContext(), MainActivity.class));
            }

            @Override
            public void onComponentChange(BaseProduct.ComponentKey componentKey, BaseComponent oldComponent, BaseComponent newComponent) { }

            @Override
            public void onInitProcess(DJISDKInitEvent djisdkInitEvent, int i) { }

            @Override
            public void onDatabaseDownloadProgress(long l, long l1) { }
        }));
    }

    @Override
    public void onPause() {
        Log.e(TAG, "onPause");
        super.onPause();
        VideoFeeder.getInstance().getPrimaryVideoFeed().removeVideoDataListener(mReceivedVideoDataListener);
        unInitVirtualStickData();
    }

    @Override
    protected void onDestroy() {
        Log.e(TAG, "onDestroy");
        super.onDestroy();

        DJIVideoStreamDecoder.getInstance().stop();
        unInitVirtualStickData();
        handlerThread.quit();
    }



    /* * * ROSACTIVITY METHOD OVERRIDES * * */
    public CameraActivity() {
        // two strings that become the title and ticker message of an Android notification. The user may tap on the notification to shut down all ROS nodes associated with the application.
        super("ROS", "ROS");
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        Log.d(TAG, "ROS Activity Init");
        // The hostname and master URI will be known from the MasterChooser activity
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(this, nodeConfiguration);
    }



    /* * * NODE_MAIN IMPLEMENTATION * * */
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("camera_node");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        compressedImagePublisher = new CompressedImagePublisher(connectedNode);
    }

    @Override
    public void onShutdown(Node node) { }

    @Override
    public void onShutdownComplete(Node node) { }

    @Override
    public void onError(Node node, Throwable throwable) { }



    /* * * CAMERAACTIVITY MEMBER FUNCTIONS * * */
    private void initUI() {
        findViewById(R.id.btn_move_left).setOnClickListener((View v) ->
                flyLeft());
        findViewById(R.id.btn_move_right).setOnClickListener((View v) ->
                flyRight());
        findViewById(R.id.btn_take_off).setOnClickListener((View v) ->
                mFlightController.startTakeoff(djiError -> showToast((djiError == null) ? "Take off Success" : djiError.getDescription())));
        findViewById(R.id.btn_land).setOnClickListener((View v) ->
                mFlightController.startLanding(djiError -> showToast((djiError == null) ? "Start Landing" : djiError.getDescription())));
        mSurfaceView = findViewById(R.id.video_previewer_surface);
        mRTMPCamera = new RtmpCamera1(mSurfaceView, new ConnectCheckerRtmp() {
            @Override
            public void onConnectionSuccessRtmp() { }

            @Override
            public void onConnectionFailedRtmp(@NonNull String reason) {
                runOnUiThread(() -> {
                    if (mRTMPCamera.reTry(5000, reason)) {
                        showToast("Retry");
                    } else {
                        showToast("Connection failed. " + reason);
                        mRTMPCamera.stopStream();
                    }
                });

            }

            @Override
            public void onNewBitrateRtmp(long bitrate) { }

            @Override
            public void onDisconnectRtmp() { }

            @Override
            public void onAuthErrorRtmp() { }

            @Override
            public void onAuthSuccessRtmp() { }
        });
        mRTMPCamera.setReTries(10);
        mSurfaceView.getHolder().addCallback(new SurfaceHolder.Callback() {
            @Override
            public void surfaceCreated(@NonNull SurfaceHolder surfaceHolder) {
                NativeHelper.getInstance().init();
                DJIVideoStreamDecoder.getInstance().init(getApplicationContext(), surfaceHolder.getSurface());
                DJIVideoStreamDecoder.getInstance().resume();
                mRTMPCamera.startStream(STREAM_URL);
            }

            @Override
            public void surfaceChanged(@NonNull SurfaceHolder surfaceHolder, int i, int i1, int i2) {
                DJIVideoStreamDecoder.getInstance().changeSurface(surfaceHolder.getSurface());
                mRTMPCamera.startPreview();
            }

            @Override
            public void surfaceDestroyed(@NonNull SurfaceHolder surfaceHolder) {
                DJIVideoStreamDecoder.getInstance().stop();
                NativeHelper.getInstance().release();
                if (mRTMPCamera.isRecording())
                    mRTMPCamera.stopRecord();
                if (mRTMPCamera.isStreaming())
                    mRTMPCamera.stopStream();
                mRTMPCamera.stopPreview();
            }
        });
    }

    private void initVirtualStickData() {
        if (mSendVirtualStickDataTask == null)
            mSendVirtualStickDataTask = new SendVirtualStickDataTask();
        if (mSendVirtualStickDataTimer == null)
            mSendVirtualStickDataTimer = new Timer();

        mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, SendVirtualStickDataTask.TICK_RATE);
    }

    private void unInitVirtualStickData() {
        if (mSendVirtualStickDataTask != null) {
            mSendVirtualStickDataTask.cancel();
            mSendVirtualStickDataTask = null;
        }

        if (mSendVirtualStickDataTimer != null) {
            mSendVirtualStickDataTimer.cancel();
            mSendVirtualStickDataTimer.purge();
            mSendVirtualStickDataTimer = null;
        }
    }

    private void setFlightController(final FlightController flightController) {
        mFlightController = flightController;

        if (mFlightController != null) {
            mFlightController.setConnectionFailSafeBehavior(ConnectionFailSafeBehavior.HOVER, null);
            mFlightController.setSmartReturnToHomeEnabled(false, null);
            mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
            mFlightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
            mFlightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
            mFlightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
            mFlightController.setVirtualStickModeEnabled(true, djiError ->
                    showToast(djiError == null ? "Virtual Stick Mode Enabled!" : djiError.getDescription()));
        }
    }

    private void setFlightAssistant(final FlightAssistant flightAssistant) {
        mFlightAssistant = flightAssistant;

        if (mFlightAssistant != null) {
            mFlightAssistant.setVisionAssistedPositioningEnabled(true, null);
            mFlightAssistant.setCollisionAvoidanceEnabled(false, null);
            mFlightAssistant.setActiveObstacleAvoidanceEnabled(false, null);
            mFlightAssistant.setVisionDetectionStateUpdatedCallback(visionDetectionState -> {
                final ObstacleDetectionSector[] detectionSectors = (visionDetectionState != null) ? visionDetectionState.getDetectionSectors() : null;
                if (detectionSectors != null) {
                    ((TextView) findViewById(R.id.tv_sector_0)).setText(String.valueOf(detectionSectors[0].getObstacleDistanceInMeters()));
                    ((TextView) findViewById(R.id.tv_sector_1)).setText(String.valueOf(detectionSectors[1].getObstacleDistanceInMeters()));
                    ((TextView) findViewById(R.id.tv_sector_2)).setText(String.valueOf(detectionSectors[2].getObstacleDistanceInMeters()));
                    ((TextView) findViewById(R.id.tv_sector_3)).setText(String.valueOf(detectionSectors[3].getObstacleDistanceInMeters()));
                }
            });
        }
    }

    private void setGimbal(final Gimbal gimbal) {
        mGimbal = gimbal;

        //getYawRelativeToAircraftHeading TODO this will be really useful for keeping the camera locked on the wall probably
        if (mGimbal != null) {
            mGimbal.setMode(GimbalMode.FPV, null);
            mGimbal.rotate(new Rotation.Builder().mode(RotationMode.ABSOLUTE_ANGLE).pitch(30.0f).time(2).build(),
                    djiError -> showToast(djiError == null ? "Rotated!" : djiError.getDescription())
            );
        }
    }

    public void showToast(final String msg) {
        runOnUiThread(() -> Toast.makeText(CameraActivity.this, msg, Toast.LENGTH_SHORT).show());
    }

    private void flyLeft() {
        //mFlightControlData.setPitch(-0.2f); // negative pitch moves leftward
        //mFlightControlData.setRoll(0.0f); // positive roll moves forward
    }

    private void flyRight() {
        //mFlightControlData.setPitch(0.2f);
    }

    class SendVirtualStickDataTask extends TimerTask {
        public static final int TICK_RATE = 200;

        @Override
        public void run() {
            if (mFlightController != null) {
                mFlightController.sendVirtualStickFlightControlData(mFlightControlData, null);
            }
        }
    }

    public void updateWalls(final ObstacleDetectionSector[] detectionSectors) {
        DetectionEvent event = new DetectionEvent(detectionSectors);
        // Call this when sectors updated and when going into ATTI mode
        switch (mState) {
            case NoWall:
                if (event.sectorsAllEqual()) {
                    final float sectorNearest = event.sectorNearest();
                    if (sectorNearest > UPPER_IDEAL_DISTANCE) {
                        setState(State.WallAhead);
                    } else if (sectorNearest >= LOWER_IDEAL_DISTANCE && sectorNearest <= UPPER_IDEAL_DISTANCE) {
                        setState(State.StartNewVector);
                    }
                }
                break;

            case WallAhead:
                if (event.sectorsAllEqual()) {
                    final float sectorNearest = event.sectorNearest();
                    if (sectorNearest >= LOWER_IDEAL_DISTANCE && sectorNearest <= UPPER_IDEAL_DISTANCE) {
                        setState(State.StartNewVector);
                    }
                } else {
                    setState(State.NoWall);
                }
                break;

            case StartNewVector: {
                float heading = mFlightController.getCompass().getHeading();

                if (heading == mFlightControlData.getYaw()) {
                    float sectorNearest = event.sectorNearest();

                    if (sectorNearest >= UPPER_IDEAL_DISTANCE) {
                        setState(State.WallStart);
                    } else {
                        if (mTargetBearing == TargetRelativeBearing.WEST) {
                            mTargetBearing = TargetRelativeBearing.EAST;
                            setState(State.StartNewVector);
                        } else
                            setState(State.NoWall);
                    }
                }
                break;
            }

            case WallStart:
                if (mFlightController.getCompass().getHeading() == mBearing.getNorthBearing())
                    setState(State.WallFollow);
                break;

            case WallResume:
                if (mFlightController.getCompass().getHeading() == mBearing.getNorthBearing())
                    setState(State.StartNewVector);
                break;

            case WallFollow:
                if (!event.sectorsAllValid()) {
                    setState(State.NoWall);
                } else if (!event.sectorsAllEqual()) {
                    if (event.sectorNearest() >= WALL_LOST_DISTANCE) // If the min distance reading is greater than WALL_LOST_DISTANCE
                        setState(State.WallDisjointed);
                } else if (detectionSectors[0].getObstacleDistanceInMeters() > UPPER_IDEAL_DISTANCE) {
                    setState(State.WallAhead);
                }
                break;

            case WallDisjointed:
                /*
                We went disjointed if we lose the wall entirely before hitting the complete state.
                The state also requires that the new distance noticed is big enough for us to move into.
                So when we enter disjointed, we can turn 90 degrees against our preferred heading and
                move sideways about a metre (guaranteed to be okay). If we catch a wall, we go back to normal states.
                If we don't, we enter disjointed AGAIN because we've probably just found a divider.
                 */

            case WallComplete:
                if (mFlightController.getCompass().getHeading() == mFlightControlData.getYaw()) {
                    float sectorNearest = event.sectorNearest();

                    if (sectorNearest >= LOWER_IDEAL_DISTANCE && sectorNearest <= UPPER_IDEAL_DISTANCE) { // If we reached the wall
                        if (event.sectorsAllEqual()) { // and if everything is equal
                            setState(State.StartNewVector);
                        } else { // and if everything is not equal
                            // Rotate to wall.
                        }
                    } else {
                        setState(State.WallResume); // Return to North Bearing
                    }
                }
                if (event.sectorsAllEqual()) {
                    setState(State.StartNewVector);
                }
                break;

        }
    }

    public void setState(final State state) {
        switch (state) {
            case NoWall:
                mBearing = null;

                mFlightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                mFlightControlData.setYaw(5.0f);
                mFlightControlData.setPitch(0.0f);
                mFlightControlData.setRoll(0.0f);
                mFlightControlData.setVerticalThrottle(0.0f);
                break;

            case WallAhead:
                mFlightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                mFlightControlData.setYaw(0.0f);
                mFlightControlData.setPitch(0.0f);
                mFlightControlData.setRoll(0.1f);
                mFlightControlData.setVerticalThrottle(0.0f);
                break;

            case StartNewVector:
                mBearing = new Bearing(mFlightController.getCompass().getHeading());

                mFlightController.setYawControlMode(YawControlMode.ANGLE);
                mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                mFlightControlData.setPitch(0.0f);
                mFlightControlData.setRoll(0.0f);
                mFlightControlData.setVerticalThrottle(0.0f);

                if (mTargetBearing == TargetRelativeBearing.WEST)
                    mFlightControlData.setYaw(mBearing.getWestBearing());
                else if (mTargetBearing == TargetRelativeBearing.EAST)
                    mFlightControlData.setYaw(mBearing.getEastBearing());
                break;

            case WallStart:
            case WallResume:
                mFlightController.setYawControlMode(YawControlMode.ANGLE);
                mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                mFlightControlData.setYaw(mBearing.getNorthBearing());
                mFlightControlData.setPitch(0.0f);
                mFlightControlData.setRoll(0.0f);
                mFlightControlData.setVerticalThrottle(0.0f);
                break;

            case WallFollow:
                mFlightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                mFlightController.setRollPitchControlMode(RollPitchControlMode.ANGLE);
                mFlightControlData.setYaw(0.0f);
                mFlightControlData.setRoll(0.0f);
                mFlightControlData.setVerticalThrottle(0.0f);

                if (mTargetBearing == TargetRelativeBearing.WEST)
                    mFlightControlData.setPitch(-0.1f);
                else if (mTargetBearing == TargetRelativeBearing.EAST)
                    mFlightControlData.setPitch(0.1f);
                else
                    mFlightControlData.setPitch(0.0f); // TODO something bad happened.
                break;

            case WallDisjointed:
                mFlightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                mFlightController.setRollPitchControlMode(RollPitchControlMode.ANGLE);
                mFlightControlData.setYaw(0.0f);
                mFlightControlData.setPitch(0.0f);
                mFlightControlData.setRoll(0.0f); // Was 0.1
                mFlightControlData.setVerticalThrottle(0.0f);
                break;

            case WallComplete:
                mFlightController.setYawControlMode(YawControlMode.ANGLE);
                mFlightController.setRollPitchControlMode(RollPitchControlMode.ANGLE);
                mFlightControlData.setPitch(0.0f);
                mFlightControlData.setRoll(0.0f);
                mFlightControlData.setVerticalThrottle(0.0f);

                if (mTargetBearing == TargetRelativeBearing.WEST)
                    mFlightControlData.setYaw(mBearing.getWestBearing());
                else if (mTargetBearing == TargetRelativeBearing.EAST)
                    mFlightControlData.setYaw(mBearing.getEastBearing());
        }
        mState = state;
    }
}