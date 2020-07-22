package com.dji.djisimulatordemo;

import android.app.Activity;
import android.content.Intent;
import android.graphics.PointF;
import android.graphics.SurfaceTexture;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.TextureView;
import android.view.TextureView.SurfaceTextureListener;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.common.flightcontroller.ConnectionFailSafeBehavior;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.ObstacleDetectionSector;
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
import dji.sdk.flighthub.FlightHubManager;
import dji.sdk.flighthub.model.RealTimeFlightData;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;
import dji.sdk.useraccount.UserAccountManager;
import dji.waypointv2.natives.data.DataProvider;

import static dji.common.flightcontroller.virtualstick.Limits.YAW_CONTROL_MAX_ANGLE;
import static dji.common.flightcontroller.virtualstick.Limits.YAW_CONTROL_MIN_ANGLE;
import static java.lang.Math.round;

public class CameraActivity extends Activity implements SurfaceTextureListener {
    private static final String TAG = CameraActivity.class.getName();
    final static float LOWER_IDEAL_DISTANCE = 0.87f;
    final static float UPPER_IDEAL_DISTANCE = 1.22f;
    final static float WALL_LOST_DISTANCE = 2.2f;

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

    private static class FlightPlan {


        public PointF targetPoint;
    }

    public class Bearing {
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

        private float mEastBearing, mNorthBearing, mWestBearing;
    }

    enum TargetRelativeBearing {
        NORTH, EAST, SOUTH, WEST
    }

    protected DJICodecManager mCodecManager;
    private FlightController mFlightController;
    private FlightAssistant mFlightAssistant;
    private Gimbal mGimbal;
    private FlightControlData mFlightControlData;


    private Timer mSendVirtualStickDataTimer;
    private SendVirtualStickDataTask mSendVirtualStickDataTask;
    protected TextureView mVideoSurface;
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener;

    private TargetRelativeBearing mTargetBearing = TargetRelativeBearing.WEST;
    private Bearing mBearing = null;
    private State mState = State.NoWall;
    private PointF mCurrentPosition, mTargetPosition;
    private FlightControllerState lastState;

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
            public void onComponentChange(BaseProduct.ComponentKey componentKey, BaseComponent oldComponent, BaseComponent newComponent) { }

            @Override
            public void onInitProcess(DJISDKInitEvent djisdkInitEvent, int i) { }

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

    private void setFlightController(final FlightController flightController) {
        mFlightController = flightController;

        if (mFlightController != null) {
            mFlightController.setConnectionFailSafeBehavior(ConnectionFailSafeBehavior.HOVER, null);
            mFlightController.setSmartReturnToHomeEnabled(false, null);
            mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
            mFlightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
            mFlightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
            mFlightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
            /*mFlightController.setStateCallback((@NonNull FlightControllerState state) -> {
                // Called 10 times per second.
                final float displacementX = (state.getVelocityX() + (lastState != null ? lastState.getVelocityX() : 0.0f)) * 0.05f;
                final float displacementY = (state.getVelocityY() + (lastState != null ? lastState.getVelocityY() : 0.0f)) * 0.05f;
                if (mCurrentPosition == null)
                    mCurrentPosition = new PointF();
                final float forwardHeading = mFlightController.getCompass().getHeading();
                final float sidewardHeading = (forwardHeading > 0) ? forwardHeading + 90.0f : forwardHeading - 90.0f;
                mCurrentPosition.x += Math.cos(forwardHeading) * displacementX + Math.cos(sidewardHeading) * displacementY;
                mCurrentPosition.y += Math.sin(forwardHeading) * displacementX + Math.sin(sidewardHeading) * displacementY;
                Log.d(TAG, String.format("X: %f, Y: %f, Dir: %f", state.getVelocityX(), state.getVelocityY(), forwardHeading));
                // TODO Y velocity showing as 0 in sim. Need to test out for real.
                //mCurrentPosition.x += (state.getVelocityX() + (lastState != null ? lastState.getVelocityX() : 0.0f)) * 0.05f;
                lastState = state;

                if (mCurrentPosition.x >= 1.0f || mCurrentPosition.y >= 1.0f) {
                    mFlightControlData.setPitch(0.0f);
                    mFlightControlData.setRoll(0.1f);
                    mFlightControlData.setYaw(0.0f);
                    mFlightControlData.setVerticalThrottle(0.0f);
                }
            });

             */

            mFlightController.setVirtualStickModeEnabled(true, djiError ->
                    showToast(djiError == null ? "Virtual Stick Mode Enabled!" : djiError.getDescription()));
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

    private void setFlightAssistant(final FlightAssistant flightAssistant) {
        mFlightAssistant = flightAssistant;

        if (mFlightAssistant != null) {
            mFlightAssistant.setVisionAssistedPositioningEnabled(true, null);
            mFlightAssistant.setCollisionAvoidanceEnabled(false, null);
            mFlightAssistant.setActiveObstacleAvoidanceEnabled(false, null);
            mFlightAssistant.setVisionDetectionStateUpdatedCallback(visionDetectionState -> {
                Log.d(TAG, visionDetectionState.isDisabled() ? "Disabled" : "Enabled");
                Log.d(TAG, visionDetectionState.isSensorBeingUsed() ? "Sensor being used." : "Sensor not being used.");
                final ObstacleDetectionSector[] detectionSectors = (visionDetectionState != null) ? visionDetectionState.getDetectionSectors() : null;
                if (detectionSectors != null) {
                    //updateWalls(detectionSectors);
                    /*
                    ((TextView) findViewById(R.id.tv_sector_0)).setText(String.valueOf(detectionSectors[0].getWarningLevel()));
                    ((TextView) findViewById(R.id.tv_sector_1)).setText(String.valueOf(detectionSectors[1].getWarningLevel()));
                    ((TextView) findViewById(R.id.tv_sector_2)).setText(String.valueOf(detectionSectors[2].getWarningLevel()));
                    ((TextView) findViewById(R.id.tv_sector_3)).setText(String.valueOf(detectionSectors[3].getWarningLevel()));
                     */
                    ((TextView) findViewById(R.id.tv_sector_0)).setText(String.valueOf(detectionSectors[0].getObstacleDistanceInMeters()));
                    ((TextView) findViewById(R.id.tv_sector_1)).setText(String.valueOf(detectionSectors[1].getObstacleDistanceInMeters()));
                    ((TextView) findViewById(R.id.tv_sector_2)).setText(String.valueOf(detectionSectors[2].getObstacleDistanceInMeters()));
                    ((TextView) findViewById(R.id.tv_sector_3)).setText(String.valueOf(detectionSectors[3].getObstacleDistanceInMeters()));
                } else {
                    Log.d(TAG, "Vision detectors null.");
                }
            });
        }
    }

    private void setGimbal(final Gimbal gimbal) {
        mGimbal = gimbal;

        //getYawRelativeToAircraftHeading TODO this will be really useful for keeping the camera locked on the wall probably
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
        //mFlightControlData.setPitch(-0.2f);
        //mFlightControlData.setRoll(0.0f); // positive roll moves forward

        mFlightController.setYawControlMode(YawControlMode.ANGLE);
        mFlightControlData.setYaw(150.0f);
        //mFlightControlData.setVerticalThrottle(0.0f);

        if (mSendVirtualStickDataTimer == null) {
            mSendVirtualStickDataTask = new SendVirtualStickDataTask();
            mSendVirtualStickDataTimer = new Timer();
            mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, SendVirtualStickDataTask.TICK_RATE);
        }
    }

    private void flyRight() {
        if (mCurrentPosition != null) {
            mCurrentPosition.x = 0;
            mCurrentPosition.y = 0;
        }
        mFlightControlData.setPitch(0.0f);
        mFlightControlData.setRoll(0.1f);
        mFlightControlData.setYaw(0);
        mFlightControlData.setVerticalThrottle(0.0f);

        if (mSendVirtualStickDataTimer == null) {
            mSendVirtualStickDataTask = new SendVirtualStickDataTask();
            mSendVirtualStickDataTimer = new Timer();
            mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, SendVirtualStickDataTask.TICK_RATE);
        }
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
                        mTargetPosition = new PointF(mCurrentPosition.x + (float) Math.cos(heading) * sectorNearest, mCurrentPosition.y + (float) Math.sin(heading) * sectorNearest);
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
                // If this is the first vector, set current position as the origin point;
                if (mCurrentPosition == null)
                    mCurrentPosition = new PointF(0.0f, 0.0f);

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