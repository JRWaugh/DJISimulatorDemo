package com.dji.djisimulatordemo;

import android.app.Activity;
import android.graphics.SurfaceTexture;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.TextureView;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.TextureView.SurfaceTextureListener;
import android.widget.Button;
import dji.sdk.useraccount.UserAccountManager;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.useraccount.UserAccountState;
import dji.common.util.CommonCallbacks;
import dji.common.product.Model;
import java.util.Timer;
import java.util.TimerTask;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.flightcontroller.FlightAssistant;
import dji.common.flightcontroller.ObstacleDetectionSector;
import dji.common.error.DJIError;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.products.Aircraft;

public class CameraActivity extends Activity implements SurfaceTextureListener,OnClickListener{
    private static final String TAG = CameraActivity.class.getName();
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;

    // Codec for video live view
    protected DJICodecManager mCodecManager = null;

    private FlightController mFlightController;
    private Timer mSendVirtualStickDataTimer;
    private SendVirtualStickDataTask mSendVirtualStickDataTask;

    private float mPitch;
    private float mRoll;
    private float mYaw;
    private float mThrottle;

    protected TextureView mVideoSurface = null;
    private TextView recordingTime;
    private TextView mSector0;
    private TextView mSector1;
    private TextView mSector2;
    private TextView mSector3;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);

        initUI();

        // The callback for receiving the raw H264 video data for camera live view
        mReceivedVideoDataListener = (videoBuffer, size) -> {
            if (mCodecManager != null)
                mCodecManager.sendDataToDecoder(videoBuffer, size);
        };

        Camera camera = DJISimulatorApplication.getCameraInstance();

        if (camera != null) {
            camera.setSystemStateCallback(cameraSystemState -> {
                if (null != cameraSystemState) {
                    final int recordTime = cameraSystemState.getCurrentVideoRecordingTimeInSeconds();
                    final int minutes = (recordTime % 3600) / 60;
                    final int seconds = recordTime % 60;
                    final boolean isVideoRecording = cameraSystemState.isRecording();

                    this.runOnUiThread(() -> {
                        recordingTime.setText(String.format("%02d:%02d", minutes, seconds));
                        // Update recordingTime TextView visibility and mRecordBtn's check state
                        if (isVideoRecording)
                            recordingTime.setVisibility(View.VISIBLE);
                        else
                            recordingTime.setVisibility(View.INVISIBLE);
                    });
                }
            });
        }
    }

    private void loginAccount() {
        UserAccountManager.getInstance().logIntoDJIUserAccount(this,
                new CommonCallbacks.CompletionCallbackWith<UserAccountState>() {
                    @Override
                    public void onSuccess(final UserAccountState userAccountState) {
                        Log.e(TAG, "Login Success");
                    }
                    @Override
                    public void onFailure(DJIError error) {
                        showToast("Login Error:"
                                + error.getDescription());
                    }
                });
    }

    @Override
    public void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();
        initFlightController();
        initPreviewer();
        loginAccount();

        if (mVideoSurface == null)
            Log.e(TAG, "mVideoSurface is null");
    }

    @Override
    public void onPause() {
        Log.e(TAG, "onPause");
        uninitPreviewer();
        super.onPause();
    }

    @Override
    public void onStop() {
        Log.e(TAG, "onStop");
        super.onStop();
    }

    @Override
    protected void onDestroy() {
        Log.e(TAG, "onDestroy");
        uninitPreviewer();

        if (null != mSendVirtualStickDataTimer) {
            mSendVirtualStickDataTask.cancel();
            mSendVirtualStickDataTask = null;
            mSendVirtualStickDataTimer.cancel();
            mSendVirtualStickDataTimer.purge();
            mSendVirtualStickDataTimer = null;
        }
        super.onDestroy();
    }

    private void initFlightController() {
        Aircraft aircraft = DJISimulatorApplication.getAircraftInstance();

        if (aircraft == null || !aircraft.isConnected()) {
            showToast("Disconnected");
            mFlightController = null;
        } else {
            mFlightController = aircraft.getFlightController();
            mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
            mFlightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
            mFlightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
            mFlightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
            mFlightController.setVirtualStickModeEnabled(true, djiError -> {
                if (djiError != null)
                    showToast(djiError.getDescription());
                else
                    showToast("Virtual Stick Mode Enabled!");
            });

            final FlightAssistant flightAssistant = mFlightController.getFlightAssistant();

            if (flightAssistant != null) {
                flightAssistant.setVisionDetectionStateUpdatedCallback(visionDetectionState -> {
                    //double minObstacleDistance = 0.0;

                    final ObstacleDetectionSector[] detectionSectors = (visionDetectionState != null) ? visionDetectionState.getDetectionSectors() : null;
                    if (detectionSectors != null) {
                        ((TextView) findViewById(R.id.tv_sector_0)).setText(String.valueOf(detectionSectors[0].getObstacleDistanceInMeters()));
                        ((TextView) findViewById(R.id.tv_sector_1)).setText(String.valueOf(detectionSectors[1].getObstacleDistanceInMeters()));
                        ((TextView) findViewById(R.id.tv_sector_2)).setText(String.valueOf(detectionSectors[2].getObstacleDistanceInMeters()));
                        ((TextView) findViewById(R.id.tv_sector_3)).setText(String.valueOf(detectionSectors[3].getObstacleDistanceInMeters()));

                    }
                    //minObstacleDistance = minObstacleDistance == 0 ? detectionSector.getObstacleDistanceInMeters() : Math.min(minObstacleDistance, detectionSector.getObstacleDistanceInMeters());
                });
            } else {
                showToast("Flight Assistant not working.");
            }
        }
    }

    private void initUI() {
        recordingTime = (TextView) findViewById(R.id.timer);
        recordingTime.setVisibility(View.INVISIBLE);
        ((Button) findViewById(R.id.btn_move_left)).setOnClickListener(this);
        ((Button) findViewById(R.id.btn_move_right)).setOnClickListener(this);
        ((Button) findViewById(R.id.btn_take_off)).setOnClickListener(this);
        ((Button) findViewById(R.id.btn_land)).setOnClickListener(this);
        ((TextureView) findViewById(R.id.video_previewer_surface)).setSurfaceTextureListener(this);
    }

    private void initPreviewer() {
        BaseProduct product = DJISimulatorApplication.getProductInstance();

        if (product == null || !product.isConnected()) {
            showToast(getString(R.string.disconnected));
        } else {
            if (null != mVideoSurface)
                mVideoSurface.setSurfaceTextureListener(this);

            if (!product.getModel().equals(Model.UNKNOWN_AIRCRAFT))
                VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(mReceivedVideoDataListener);
        }
    }

    private void uninitPreviewer() {
        Camera camera = DJISimulatorApplication.getCameraInstance();

        if (camera != null) // Reset the callback
            VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(null);
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        Log.e(TAG, "onSurfaceTextureAvailable");
        if (mCodecManager == null)
            mCodecManager = new DJICodecManager(this, surface, width, height);
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        Log.e(TAG, "onSurfaceTextureSizeChanged");
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        Log.e(TAG,"onSurfaceTextureDestroyed");
        if (mCodecManager != null) {
            mCodecManager.cleanSurface();
            mCodecManager = null;
        }

        return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) { }

    public void showToast(final String msg) {
        runOnUiThread(() -> Toast.makeText(CameraActivity.this, msg, Toast.LENGTH_SHORT).show());
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.btn_move_left:
                flyLeft();
                break;

            case R.id.btn_move_right:
                flyRight();
                break;

            case R.id.btn_take_off:
                mFlightController.startTakeoff(djiError -> {
                    if (djiError != null)
                        showToast(djiError.getDescription());
                    else
                        showToast("Take off Success");
                });
                break;

            case R.id.btn_land:
                mFlightController.startLanding(djiError -> {
                    if (djiError != null)
                        showToast(djiError.getDescription());
                    else
                        showToast("Start Landing");
                });
                break;

            default:
                break;
        }
    }

     private void flyLeft() {
         mPitch = -2.0f;
         mRoll = 0;
         mYaw = 0;
         mThrottle = 0;

         if (null == mSendVirtualStickDataTimer) {
             mSendVirtualStickDataTask = new CameraActivity.SendVirtualStickDataTask();
             mSendVirtualStickDataTimer = new Timer();
             mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 100, 200);
         }
     }

    private void flyRight() {
        mPitch = 2.0f;
        mRoll = 0;
        mYaw = 0;
        mThrottle = 0;

        if (null == mSendVirtualStickDataTimer) {
            mSendVirtualStickDataTask = new CameraActivity.SendVirtualStickDataTask();
            mSendVirtualStickDataTimer = new Timer();
            mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, 200);
        }
    }

    class SendVirtualStickDataTask extends TimerTask {
        @Override
        public void run() {
            if (mFlightController != null)
                mFlightController.sendVirtualStickFlightControlData(new FlightControlData(mPitch, mRoll, mYaw, mThrottle), djiError -> {});
        }
    }
}