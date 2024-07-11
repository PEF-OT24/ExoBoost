package com.example;

import java.util.List;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;

public final class PythonScanCallback extends ScanCallback {
    public interface Interface {
        void onScanFailed(int errorCode);
        void onScanResult(int callbackType, ScanResult result);
        void onBatchScanResults(List<ScanResult> results);
    }

    private Interface callback;

    public PythonScanCallback(Interface pythonCallback) {
        this.callback = pythonCallback;
    }

    @Override
    public void onScanFailed(int errorCode) {
        callback.onScanFailed(errorCode);
    }

    @Override
    public void onScanResult(int callbackType, ScanResult result) {
        callback.onScanResult(callbackType, result);
    }

    @Override
    public void onBatchScanResults(List<ScanResult> results) {
        callback.onBatchScanResults(results);
    }
}
