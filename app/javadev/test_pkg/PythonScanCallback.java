package javadev.test_pkg;

import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import java.util.List;

public class PythonScanCallback extends ScanCallback {

    // Constructor vacío
    public PythonScanCallback() {
        super();
        System.out.println("Clase creada en java");
    }

    @Override
    public void onScanFailed(int errorCode) {
        // Implementación personalizada del método        
        System.out.println("Scan failed with error code: " + errorCode);
        System.out.println("Método onScanFailed");
    }
    
    @Override
    public void onScanResult(int callbackType, ScanResult result) {
        // Implementación personalizada del método
        System.out.println("Scan result: " + result);
        System.out.println("Método onScanResult");
    }
    
    @Override
    public void onBatchScanResults(List<ScanResult> results) {
        // Implementación personalizada del método
        System.out.println("Batch scan results: " + results);
        System.out.println("Método onBatchScanResults");
    }
}
