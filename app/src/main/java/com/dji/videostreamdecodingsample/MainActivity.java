package com.dji.videostreamdecodingsample;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.util.ArrayList;
import java.util.List;

import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;

public class MainActivity extends Activity {
    private static final String TAG = MainActivity.class.getName();
    private static final String[] REQUIRED_PERMISSION_LIST = new String[] {
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
    private BaseProduct mBaseProduct = null;
    private final List<String> missingPermission = new ArrayList<>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        for (final String permission : REQUIRED_PERMISSION_LIST)
            if (ContextCompat.checkSelfPermission(this, permission) != PackageManager.PERMISSION_GRANTED)
                missingPermission.add(permission);

        if (missingPermission.isEmpty())
            startSDKRegistration();
        else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M)
            ActivityCompat.requestPermissions(this, missingPermission.toArray(new String[0]), REQUEST_PERMISSION_CODE);

        initUI();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        // Check for granted permission and remove from missing list
        if (requestCode == REQUEST_PERMISSION_CODE)
            for (int i = grantResults.length - 1; i >= 0; --i)
                if (grantResults[i] == PackageManager.PERMISSION_GRANTED)
                    missingPermission.remove(permissions[i]);

        // If permissions are given, start registration
        if (missingPermission.isEmpty())
            startSDKRegistration();
        else
            showToast("Missing permissions.");
    }

    public void startSDKRegistration() {
        showToast("Registering, please wait...");
        DJISDKManager.getInstance().registerApp(getApplicationContext(), new DJISDKManager.SDKManagerCallback() {
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
            public void onProductConnect(BaseProduct baseProduct) {
                Log.d(TAG, String.format("onProductConnect: %s", baseProduct.toString()));
                mBaseProduct = baseProduct;
                onStatusChanged();
            }

            @Override
            public void onProductDisconnect() {
                Log.d(TAG, "onProductDisconnect");
                mBaseProduct = null;
                onStatusChanged();
            }

            @Override
            public void onProductChanged(BaseProduct baseProduct) {
                Log.d(TAG, String.format("onProductChanged: %s", baseProduct.toString()));
                mBaseProduct = baseProduct;
                onStatusChanged();
            }

            @Override
            public void onComponentChange(BaseProduct.ComponentKey componentKey, BaseComponent oldComponent, BaseComponent newComponent) {
                Log.d(TAG, String.format("onComponentChange key: %s, oldComponent:%s, newComponent: %s", componentKey, oldComponent, newComponent));

                if (newComponent != null)
                    newComponent.setComponentListener(isConnected -> onStatusChanged());

                onStatusChanged();
            }

            @Override
            public void onInitProcess(DJISDKInitEvent djisdkInitEvent, int i) { }

            @Override
            public void onDatabaseDownloadProgress(long l, long l1) { }
        });
    }

    private void initUI() {
        TextView mTextConnectionStatus = findViewById(R.id.text_connection_status);
        Button mBtnOpen = findViewById(R.id.btn_open);
        mBtnOpen.setOnClickListener(v -> startActivity(new Intent(this, CameraActivity.class)));
        ((TextView) findViewById(R.id.text_sdk_version)).setText(getString(R.string.sdk_version, DJISDKManager.getInstance().getSDKVersion()));

        if (mBaseProduct != null) {
            if (mBaseProduct.getModel() != null) { // RC and Drone both connected
                mBtnOpen.setEnabled(true);
                mTextConnectionStatus.setText(getString(R.string.connection_status, mBaseProduct.getModel().getDisplayName()));
            } else { // Only RC connected
                mBtnOpen.setEnabled(false);
                mTextConnectionStatus.setText(getString(R.string.connection_status, "Handheld"));
            }
        } else { // RC not connected and Drone unknowable
            mBtnOpen.setEnabled(false);
            mTextConnectionStatus.setText(getString(R.string.connection_status, "No Product"));
        }
    }

    private void onStatusChanged() {
        runOnUiThread(this::initUI);
    }

    private void showToast(final String msg) {
        runOnUiThread(() -> Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show());
    }
}