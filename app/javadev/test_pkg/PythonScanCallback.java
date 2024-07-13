package javadev.test_pkg;

import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import java.util.List;

public class PythonScanCallback extends ScanCallback {

    // Constructor vacío
    public CustomScanCallback() {
        super();
    }

    @Override
    public void onScanFailed(int errorCode) {
        // Implementación personalizada del método
        System.out.println("Scan failed with error code: " + errorCode);
    }

    @Override
    public void onScanResult(int callbackType, ScanResult result) {
        // Implementación personalizada del método
        System.out.println("Scan result: " + result);
    }

    @Override
    public void onBatchScanResults(List<ScanResult> results) {
        // Implementación personalizada del método
        System.out.println("Batch scan results: " + results);
    }
}
