package javadev.test_pkg;

import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import java.util.ArrayList;
import java.util.List;

public class PythonScanCallback extends ScanCallback {
    public List<ScanResult> scanResults = new ArrayList<>();
    public int errorCode = -1; // Inicializamos el código de error con un valor por defecto
    
    public PythonScanCallback() {
        super();
        System.out.println("Clase creada en java (python)");
    }

    @Override
    public void onScanFailed(int errorCode) {
        // Método que obtiene el código de error
        super.onScanFailed(errorCode);
        System.out.println("onScanFailed");
        this.errorCode = errorCode;
    }
    
    @Override
    public void onScanResult(int callbackType, ScanResult result) {
        // Método que obtiene los dispositivos escaneados
        super.onScanResult(callbackType, result);
        System.out.println("onScanResult");
        if (!scanResults.contains(result)) {
            scanResults.add(result);
        }
    }
    
    @Override
    public void onBatchScanResults(List<ScanResult> results) {
        // Implementación personalizada del método
        super.onBatchScanResults(results);
        System.out.println("Batch scan results: " + results);
        System.out.println("Método onBatchScanResults");
    }
    
    public List<ScanResult> getScanResults() {
        // Método que devuelve cada dispositivo escaneado
        System.out.println("getScanResults (python)");
        return scanResults;
    }
    
    public int getErrorCode() {
        // Método que devuelve el código de error
        System.out.println("getErrorCode");
        return errorCode;
    }
}