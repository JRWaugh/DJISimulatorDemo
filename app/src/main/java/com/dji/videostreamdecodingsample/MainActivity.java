package com.dji.videostreamdecodingsample;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.dji.videostreamdecodingsample.media.DJIVideoStreamDecoder;

import java.util.ArrayList;
import java.util.List;

import dji.common.camera.ResolutionAndFrameRate;
import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.common.useraccount.UserAccountState;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;


public class MainActivity extends Activity {
    private static final String TAG = MainActivity.class.getName();
    private static final String[] REQUIRED_PERMISSION_LIST = new String[]{
            Manifest.permission.VIBRATE,
            Manifest.permission.INTERNET,
            Manifest.permission.ACCESS_WIFI_STATE,
            Manifest.permission.WAKE_LOCK,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.ACCESS_NETWORK_STATE,
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.CHANGE_WIFI_STATE,
            Manifest.permission.WRITE_EXTERNAL_STORAGE,
            Manifest.permission.BLUETOOTH,
            Manifest.permission.BLUETOOTH_ADMIN,
            Manifest.permission.READ_EXTERNAL_STORAGE,
            Manifest.permission.READ_PHONE_STATE,
    };
    private static final int REQUEST_PERMISSION_CODE = 12345;
    private final List<String> missingPermission = new ArrayList<>();
    private TextView mTextConnectionStatus;
    private TextView mTextProduct;
    private Button mBtnOpen;
    private Handler mHandler;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mHandler = new Handler(getMainLooper());
        setContentView(R.layout.activity_main);
        mTextConnectionStatus = findViewById(R.id.text_connection_status);
        mTextProduct = findViewById(R.id.text_product_info);
        mBtnOpen = findViewById(R.id.btn_open);
        mBtnOpen.setOnClickListener(v -> startActivity(new Intent(this, CameraActivity.class)));
        mBtnOpen.setEnabled(false);
        ((TextView) findViewById(R.id.textView2)).setText(getResources().getString(R.string.sdk_version, DJISDKManager.getInstance().getSDKVersion()));
    }

    @Override
    public void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();

        checkAndRequestPermissions();
    }

    private void checkAndRequestPermissions() {
        for (final String permission : REQUIRED_PERMISSION_LIST)
            if (ContextCompat.checkSelfPermission(this, permission) != PackageManager.PERMISSION_GRANTED)
                missingPermission.add(permission);

        if (missingPermission.isEmpty()) {
            startSDKRegistration();
        } else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M)
            ActivityCompat.requestPermissions(this, missingPermission.toArray(new String[0]), REQUEST_PERMISSION_CODE);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        // Check for granted permission and remove from missing list
        if (requestCode == REQUEST_PERMISSION_CODE)
            for (int i = grantResults.length - 1; i >= 0; i--)
                if (grantResults[i] == PackageManager.PERMISSION_GRANTED)
                    missingPermission.remove(permissions[i]);

        // If permissions are given, start registration
        if (missingPermission.isEmpty())
            startSDKRegistration();
        else
            showToast("Missing permissions!!!");
    }

    public void startSDKRegistration() {
        showToast("Registering, please wait...");
        AsyncTask.execute(() -> DJISDKManager.getInstance().registerApp(getApplicationContext(), new DJISDKManager.SDKManagerCallback() {
            @Override
            public void onRegister(DJIError djiError) {
                Log.d(TAG, djiError.getDescription());
                if (djiError == DJISDKError.REGISTRATION_SUCCESS) {
                    showToast("Register Success");
                    DJISDKManager.getInstance().startConnectionToProduct();
                } else {
                    showToast("Register Fail. Check network is available.");
                }
            }

            @Override
            public void onProductDisconnect() {
                Log.d(TAG, "onProductDisconnect");
                showToast("Product Disconnected");

                mHandler.post(() -> {
                    mTextProduct.setText(R.string.product_information);
                    mTextConnectionStatus.setText(R.string.connection_loose);
                    mBtnOpen.setEnabled(false);
                });
            }

            @Override
            public void onProductConnect(BaseProduct baseProduct) {
                Log.d(TAG, String.format("onProductConnect newProduct:%s", baseProduct));
                showToast("Product Connected");

                mHandler.post(() -> {
                    mTextProduct.setText(baseProduct.getModel() != null ? baseProduct.getModel().getDisplayName() : getString(R.string.product_information));
                    mTextConnectionStatus.setText(String.format("Status: %s connected", baseProduct.isConnected() ? "DJIAircraft" : "DJIHandHeld"));
                    if (baseProduct.isConnected())
                        mBtnOpen.setEnabled(true);
                });
            }

            @Override
            public void onComponentChange(BaseProduct.ComponentKey componentKey, BaseComponent oldComponent, BaseComponent newComponent) {
                Log.d(TAG, String.format("onComponentChange key:%s, oldComponent:%s, newComponent:%s", componentKey, oldComponent, newComponent));

                if (newComponent != null) {
                    newComponent.setComponentListener(isConnected -> Log.d(TAG, "onComponentConnectivityChanged: " + isConnected));
                    DJISDKManager.getInstance().startConnectionToProduct();
                }
            }

            @Override
            public void onInitProcess(DJISDKInitEvent djisdkInitEvent, int i) {
            }

            @Override
            public void onDatabaseDownloadProgress(long l, long l1) {
            }
        }));
    }

    public void showToast(final String msg) {
        runOnUiThread(() -> Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show());
    }
}