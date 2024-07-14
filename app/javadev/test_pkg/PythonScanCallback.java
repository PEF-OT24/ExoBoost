package javadev.test_pkg;

import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.bluetooth.BluetoothDevice;
import java.util.ArrayList;
import java.util.List;

public class PythonScanCallback extends ScanCallback {
    private List<BluetoothDevice> scanResults = new ArrayList<>();
    private int errorCode = -1; // Inicializamos el código de error con un valor por defecto

    public PythonScanCallback() {
        super();
        System.out.println("Clase creada en java (python)");
    }

    @Override
    public void onScanFailed(int errorCode) {
        // Método que obtiene el código de error
        super.onScanFailed(errorCode);
        System.out.println("onScanFailed (python)");
        this.errorCode = errorCode;
    }

    @Override
    public void onScanResult(int callbackType, ScanResult result) {
        // Método que obtiene los dispositivos escaneados
        System.out.println("onScanResult (python)");
        super.onScanResult(callbackType, result);
        BluetoothDevice dispositivo = result.getDevice();
        this.scanResults.add(dispositivo);
    }

    @Override
    public void onBatchScanResults(List<ScanResult> results) {
        // Implementación personalizada del método
        super.onBatchScanResults(results);
        System.out.println("Batch scan results: (python) " + results);
        System.out.println("Método onBatchScanResults");
    }

    public List<BluetoothDevice> getScanResults() {
        // Método que devuelve cada dispositivo escaneado
        System.out.println("getScanResults (python)");
        return scanResults;
    }

    public int getErrorCode() {
        // Método que devuelve el código de error
        System.out.println("getErrorCode (python)");
        return errorCode;
    }
}