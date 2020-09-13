package com.dji.videostreamdecodingsample;

import android.content.Intent;
import android.graphics.ImageFormat;
import android.graphics.YuvImage;
import android.media.Image;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceView;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import com.dji.videostreamdecodingsample.media.DJIVideoStreamDecoder;

import org.ros.android.RosActivity;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;

import java.nio.ByteBuffer;
import java.util.Timer;
import java.util.TimerTask;

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
import dji.sdk.useraccount.UserAccountManager;
import sensor_msgs.Joy;

public class CameraActivity extends RosActivity implements NodeMain {
    private static final String TAG = CameraActivity.class.getName();
    private final int MAX_PITCH_SPEED = 4;
    private final int HALF_PITCH_SPEED = MAX_PITCH_SPEED / 2;
    private final int SLOW_PITCH_SPEED = HALF_PITCH_SPEED / 2;
    private final int MAX_ROLL_SPEED = 4;
    private final int HALF_ROLL_SPEED = MAX_ROLL_SPEED / 2;
    private final int SLOW_ROLL_SPEED = HALF_ROLL_SPEED / 2;
    private final int MAX_VERTICAL_SPEED = 2;
    private final int MAX_YAW_SPEED = 30;

    private FlightController mFlightController;
    private FlightControlData mFlightControlData;
    private VideoFeeder.VideoDataListener mReceivedVideoDataListener;

    private Timer mSendVirtualStickDataTimer;
    private SendVirtualStickDataTask mSendVirtualStickDataTask;
    private byte[] mYUV;
    private CompressedImagePublisher compressedImagePublisher;
    private long ticks = 0;


    /*  *  *  ACTIVITY METHOD OVERRIDES *  *  */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.e(TAG, "onCreate");
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);


        initUI();

        ticks = System.currentTimeMillis();

        DJIVideoStreamDecoder.getInstance().init(this, null);
        DJIVideoStreamDecoder.getInstance().setFrameListener((Image frame) -> {
            Log.d(TAG, String.format("Time since last frame: %d", System.currentTimeMillis() - ticks));
            ticks = System.currentTimeMillis();

            ByteBuffer Y = frame.getPlanes()[0].getBuffer();
            ByteBuffer U = frame.getPlanes()[1].getBuffer();
            ByteBuffer V = frame.getPlanes()[2].getBuffer();

            final int WIDTH = frame.getWidth();
            final int HEIGHT = frame.getHeight();
            final int Yb = Y.remaining();
            final int Ub = U.remaining();
            final int Vb = V.remaining();

            if (mYUV == null)
                mYUV = new byte[Yb + Ub + Vb];

            // Swap the U and V planes
            Y.get(mYUV, 0, Yb);
            V.get(mYUV, Yb, Vb);
            U.get(mYUV, Yb + Vb, Ub);

            AsyncTask.execute(() -> {
                if (compressedImagePublisher != null)
                    compressedImagePublisher.onNewImage(new YuvImage(mYUV, ImageFormat.NV21, WIDTH, HEIGHT, null));
            });
        });

        mReceivedVideoDataListener = (bytes, i) -> DJIVideoStreamDecoder.getInstance().parse(bytes, i);
        mFlightControlData = new FlightControlData(0.0f, 0.0f, 0.0f, 0.0f);
    }

    @Override
    public void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();

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

                VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(mReceivedVideoDataListener);
            }


            @Override
            public void onProductChanged(BaseProduct baseProduct) {
                setFlightController(((Aircraft) baseProduct).getFlightController());
                setFlightAssistant(((Aircraft) baseProduct).getFlightController().getFlightAssistant());
                setGimbal(baseProduct.getGimbal());
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
            public void onComponentChange(BaseProduct.ComponentKey componentKey, BaseComponent oldComponent, BaseComponent newComponent) {
            }

            @Override
            public void onInitProcess(DJISDKInitEvent djisdkInitEvent, int i) {
            }

            @Override
            public void onDatabaseDownloadProgress(long l, long l1) {
            }
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


    }


    /* * * ROSACTIVITY METHOD OVERRIDES * * */
    public CameraActivity() {
        // Two strings that become the title and ticker message of an Android notification. The user may tap on the notification to shut down all ROS nodes associated with the application.
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
        Subscriber<Joy> joySubscriber = connectedNode.newSubscriber("/joy", Joy._TYPE);

        joySubscriber.addMessageListener((joyData) -> {
            if (joyData.getButtons()[7] == 1 && joyData.getButtons()[8] == 1) { // if L2 and R2 pressed
                mFlightControlData.setPitch(SLOW_PITCH_SPEED * -joyData.getAxes()[0]);
                mFlightControlData.setRoll(SLOW_ROLL_SPEED * joyData.getAxes()[1]);
            } else if (joyData.getButtons()[7] == 1 || joyData.getButtons()[8] == 1) {
                mFlightControlData.setPitch(HALF_PITCH_SPEED * -joyData.getAxes()[0]);
                mFlightControlData.setRoll(HALF_ROLL_SPEED * joyData.getAxes()[1]);
            } else {
                mFlightControlData.setPitch(MAX_PITCH_SPEED * -joyData.getAxes()[0]);
                mFlightControlData.setRoll(MAX_ROLL_SPEED * joyData.getAxes()[1]);
            }

            mFlightControlData.setYaw(MAX_YAW_SPEED * -joyData.getAxes()[2]);
            mFlightControlData.setVerticalThrottle(MAX_VERTICAL_SPEED * joyData.getAxes()[5]);

            if (joyData.getButtons()[4] == 1 && joyData.getButtons()[5] == 1) { // if L1 and R1 are pressed
                if (mFlightController.getState().isFlying()) {
                    mFlightController.startLanding(djiError -> showToast((djiError == null) ? "Start Landing" : djiError.getDescription()));
                } else {
                    mFlightController.startTakeoff(djiError -> showToast((djiError == null) ? "Take off Success" : djiError.getDescription()));
                }
            }
        });
    }

    @Override
    public void onShutdown(Node node) {
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }


    /* * * CAMERAACTIVITY MEMBER FUNCTIONS * * */
    private void initUI() {
        findViewById(R.id.btn_take_off).setOnClickListener((View v) ->
                mFlightController.startTakeoff(djiError -> showToast((djiError == null) ? "Take off Success" : djiError.getDescription())));
        findViewById(R.id.btn_land).setOnClickListener((View v) ->
                mFlightController.startLanding(djiError -> showToast((djiError == null) ? "Start Landing" : djiError.getDescription())));

        //SurfaceView mSurfaceView = findViewById(R.id.video_previewer_surface);
    }

    private void initVirtualStickData() {
        if (mSendVirtualStickDataTimer == null) {
            mSendVirtualStickDataTimer = new Timer();

            if (mSendVirtualStickDataTask == null)
                mSendVirtualStickDataTask = new SendVirtualStickDataTask();

            mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, SendVirtualStickDataTask.TICK_RATE);
        }
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
            mFlightController.setVirtualStickModeEnabled(true, djiError -> showToast(djiError == null ? "Virtual Stick Mode Enabled!" : djiError.getDescription()));
        }
    }

    private void setFlightAssistant(final FlightAssistant flightAssistant) {
        if (flightAssistant != null) {
            flightAssistant.setVisionAssistedPositioningEnabled(true, null);
            flightAssistant.setCollisionAvoidanceEnabled(false, null);
            flightAssistant.setActiveObstacleAvoidanceEnabled(false, null);
        }


        }

    private void setGimbal(final Gimbal gimbal) {

        if (gimbal != null) {
            gimbal.setMode(GimbalMode.FPV, null);
            gimbal.rotate(new Rotation.Builder().mode(RotationMode.ABSOLUTE_ANGLE).pitch(30.0f).time(2).build(), null);
        }
    }

    public void showToast(final String msg) {
        runOnUiThread(() -> Toast.makeText(CameraActivity.this, msg, Toast.LENGTH_SHORT).show());
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
}