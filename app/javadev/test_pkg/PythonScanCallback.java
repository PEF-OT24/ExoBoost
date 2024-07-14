package javadev.test_pkg;

import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.bluetooth.BluetoothDevice;
import java.util.ArrayList;
import java.util.List;

public class PythonScanCallback extends ScanCallback {
    private final List<BluetoothDevice> scanResults = new ArrayList<>();
    private int errorCode = -1;
    private int contador = 0;

    public PythonScanCallback() {
        super();
        System.out.println("Clase creada en java (python)");
    }

    @Override
    public void onScanResult(int callbackType, ScanResult result) {
        System.out.println("onScanResult (python)");
        BluetoothDevice dispositivo = result.getDevice();
        synchronized (scanResults) {
            if (!scanResults.contains(dispositivo)) {
                scanResults.add(dispositivo);
                contador += 1;
                System.out.println("Dispositivo escaneado: (python) " + contador);
            }
        }
    }

    @Override
    public void onScanFailed(int errorCode) {
        System.out.println("onScanFailed (python)");
        this.errorCode = errorCode;
    }

    @Override
    public void onBatchScanResults(List<ScanResult> results) {
        System.out.println("Batch scan results: (python) " + results);
        synchronized (scanResults) {
            for (ScanResult result : results) {
                BluetoothDevice dispositivo = result.getDevice();
                if (!scanResults.contains(dispositivo)) {
                    scanResults.add(dispositivo);
                }
            }
            contador = scanResults.size();
        }
    }

    public List<BluetoothDevice> getScanResults() {
        System.out.println("getScanResults (python)" + scanResults);
        synchronized (scanResults) {
            return new ArrayList<>(scanResults);
        }
    }

    public int getErrorCode() {
        System.out.println("getErrorCode (python)");
        return errorCode;
    }
}
