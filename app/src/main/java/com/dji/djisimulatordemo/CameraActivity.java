package com.dji.djisimulatordemo;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.SurfaceTexture;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.TextureView;
import android.view.TextureView.SurfaceTextureListener;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.Timer;
import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.common.flightcontroller.ConnectionFailSafeBehavior;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.ObstacleDetectionSector;
import dji.common.flightcontroller.ObstacleDetectionSectorWarning;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.gimbal.GimbalMode;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.product.Model;
import dji.common.useraccount.UserAccountState;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.flightcontroller.FlightAssistant;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;
import dji.sdk.useraccount.UserAccountManager;

public class CameraActivity extends Activity implements SurfaceTextureListener {
    private static final String TAG = CameraActivity.class.getName();

    protected DJICodecManager mCodecManager;
    private FlightController mFlightController;
    private FlightAssistant mFlightAssistant;
    private Gimbal mGimbal;
    private FlightControlData mFlightControlData;

    private Timer mSendVirtualStickDataTimer;
    private SendVirtualStickDataTask mSendVirtualStickDataTask;
    protected TextureView mVideoSurface;
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);

        initUI();
        mReceivedVideoDataListener = (videoBuffer, size) -> {
            if (mCodecManager != null) {
                mCodecManager.sendDataToDecoder(videoBuffer, size);
            }
        };
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
                    loginAccount();
                    DJISDKManager.getInstance().startConnectionToProduct();
                }
            }

            @Override
            public void onProductConnect(BaseProduct baseProduct) {
                Log.d(TAG, String.format("onProductConnect newProduct:%s", baseProduct));
                showToast("Product Connected");

                setPreviewer(mReceivedVideoDataListener);
                if (!baseProduct.getModel().equals(Model.UNKNOWN_AIRCRAFT)) {
                    setFlightController(((Aircraft) baseProduct).getFlightController());
                    setFlightAssistant(((Aircraft) baseProduct).getFlightController().getFlightAssistant());
                }
                setGimbal(baseProduct.getGimbal());
                initVirtualStickData();
            }

            @Override
            public void onProductDisconnect() {
                Log.d(TAG, "onProductDisconnect");

                setPreviewer(mReceivedVideoDataListener);
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

        setPreviewer(null);
        unInitVirtualStickData();
    }

    @Override
    public void onStop() {
        Log.e(TAG, "onStop");
        super.onStop();
    }

    @Override
    protected void onDestroy() {
        Log.e(TAG, "onDestroy");
        super.onDestroy();

        setPreviewer(null);
        unInitVirtualStickData();
    }

    private void initUI() {
        mVideoSurface = findViewById(R.id.video_previewer_surface);
        mVideoSurface.setSurfaceTextureListener(this);

        findViewById(R.id.btn_move_left).setOnClickListener((View v) ->
                flyLeft());
        findViewById(R.id.btn_move_right).setOnClickListener((View v) ->
                flyRight());
        findViewById(R.id.btn_take_off).setOnClickListener((View v) ->
                mFlightController.startTakeoff(djiError -> showToast((djiError == null) ? "Take off Success" : djiError.getDescription())));
        findViewById(R.id.btn_land).setOnClickListener((View v) ->
                mFlightController.startLanding(djiError -> showToast((djiError == null) ? "Start Landing" : djiError.getDescription())));
    }

    private void startMission() {

    }

    private void setFlightController(final FlightController flightController) {
        mFlightController = flightController;

        if (mFlightController != null) {
            mFlightController.setStateCallback((FlightControllerState flightControllerState) -> {
                flightControllerState.getFlightMode();
                flightControllerState.isFlying();
            });
            mFlightController.setConnectionFailSafeBehavior(ConnectionFailSafeBehavior.HOVER, null);
            mFlightController.setSmartReturnToHomeEnabled(false, null);
            mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
            mFlightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
            mFlightController.setYawControlMode(YawControlMode.ANGLE);
            mFlightController.setVerticalControlMode(VerticalControlMode.POSITION);

            mFlightController.setVirtualStickModeEnabled(true, djiError ->
                    showToast(djiError == null ? "Virtual Stick Mode Enabled!" : djiError.getDescription()));
        }
    }

    private void initVirtualStickData() {
        mSendVirtualStickDataTimer = new Timer();
        mSendVirtualStickDataTask = new CameraActivity.SendVirtualStickDataTask();
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

    private void setFlightAssistant(final FlightAssistant flightAssistant) {
        mFlightAssistant = flightAssistant;

        if (mFlightAssistant != null) {
            mFlightAssistant.setVisionAssistedPositioningEnabled(true, null);
            mFlightAssistant.setCollisionAvoidanceEnabled(false, null);
            mFlightAssistant.setActiveObstacleAvoidanceEnabled(false, null);
            mFlightAssistant.setVisionDetectionStateUpdatedCallback(visionDetectionState -> {
                final ObstacleDetectionSector[] detectionSectors = (visionDetectionState != null) ? visionDetectionState.getDetectionSectors() : null;
                if (detectionSectors != null) {
                    ((TextView) findViewById(R.id.tv_sector_0)).setText(String.valueOf(detectionSectors[0].getWarningLevel()));
                    ((TextView) findViewById(R.id.tv_sector_1)).setText(String.valueOf(detectionSectors[1].getWarningLevel()));
                    ((TextView) findViewById(R.id.tv_sector_2)).setText(String.valueOf(detectionSectors[2].getWarningLevel()));
                    ((TextView) findViewById(R.id.tv_sector_3)).setText(String.valueOf(detectionSectors[3].getWarningLevel()));
                }
            });
        }
    }

    private void setGimbal(final Gimbal gimbal) {
        mGimbal = gimbal;

        if (mGimbal != null) {
            mGimbal.setMode(GimbalMode.FPV, djiError -> showToast(djiError == null ? "Mode set!" : djiError.getDescription()));
            mGimbal.rotate(new Rotation.Builder().mode(RotationMode.ABSOLUTE_ANGLE).pitch(30.0f).time(2).build(),
                    djiError -> showToast(djiError == null ? "Rotated!" : djiError.getDescription())
            );
        }
    }

    private void setPreviewer(VideoFeeder.VideoDataListener receivedVideoDataListener) {
        VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(receivedVideoDataListener);
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
        Log.e(TAG, "onSurfaceTextureDestroyed");
        if (mCodecManager != null) {
            mCodecManager.cleanSurface();
            mCodecManager = null;
        }
        return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        /*
        Handler handler = new Handler(getMainLooper());
        handler.post(() -> {
            Mat input = new Mat(), gray = new Mat(), filtered = new Mat(), output = new Mat();
            Bitmap bmp = mVideoSurface.getBitmap();
            Utils.bitmapToMat(bmp, input);
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
            Imgproc.bilateralFilter(gray, filtered, 7, 50, 50);
            Imgproc.Canny(filtered, output, 80, 90);
            Utils.matToBitmap(output, bmp);
        });

         */
        //((ImageView) findViewById(R.id.img)).setImageBitmap(bmp);
    }

    public void showToast(final String msg) {
        runOnUiThread(() -> Toast.makeText(CameraActivity.this, msg, Toast.LENGTH_SHORT).show());
    }

    private void flyLeft() {
        mFlightControlData.setPitch(-2.0f);
        mFlightControlData.setRoll(0.0f); // positive roll moves forward

        mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, 200);
    }

    private void flyRight() {
        mFlightControlData.setPitch(2.0f);
        mFlightControlData.setRoll(0.0f);

        mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, 200);
    }

    private void loginAccount() {
        UserAccountManager.getInstance().logIntoDJIUserAccount(this, new CommonCallbacks.CompletionCallbackWith<UserAccountState>() {
            public void onSuccess(final UserAccountState userAccountState) {
                Log.e(TAG, "Login Success");
            }

            public void onFailure(DJIError error) {
                showToast("Login Error:" + error.getDescription());
            }
        });
    }

    class SendVirtualStickDataTask extends TimerTask {
        @Override
        public void run() {
            if (mFlightController != null)
                mFlightController.sendVirtualStickFlightControlData(mFlightControlData, djiError -> {
                });
        }
    }
}