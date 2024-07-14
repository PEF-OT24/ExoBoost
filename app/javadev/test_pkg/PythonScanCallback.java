package javadev.test_pkg;

import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.bluetooth.BluetoothDevice;
import java.util.ArrayList;
import java.util.List;

public class PythonScanCallback extends ScanCallback {
    private List<BluetoothDevice> scanResults = new ArrayList<>();
    private List<String> nombres = new ArrayList<>();
    private int errorCode = -1; // Inicializamos el código de error con un valor por defecto
    private int contador = 0;

    public PythonScanCallback() {
        super();
        System.out.println("Objeto de ScanCallback creado en java (python)");
    }

    @Override
    public void onScanResult(int callbackType, ScanResult result) {
        // Método que obtiene los dispositivos escaneados
        super.onScanResult(callbackType, result);
        System.out.println("onScanResult (python)");

        // Se obteien el dispositivo
        BluetoothDevice dispositivo = result.getDevice();
        String nombre = dispositivo.getName();

        if (nombre != null) {
            // Se agrega el dispositivo a la lista
            this.scanResults.add(dispositivo);
            System.out.println("Dispositivo escaneado: (python) " + this.contador);
            this.contador += 1;

            // Se obtiene el nombre
            this.nombres.add(nombre);

            // Se imprimen los datos
            System.out.println("Nombre de dispositivo: (python) " + nombre);
            System.out.println("Dispositivos" + this.scanResults.size());
            System.out.println("------------------------");
        }
    }

    @Override
    public void onScanFailed(int errorCode) {
        // Método que obtiene el código de error
        super.onScanFailed(errorCode);
        System.out.println("onScanFailed (python)");
        this.errorCode = errorCode;
    }

    @Override
    public void onBatchScanResults(List<ScanResult> results) {
        // Implementación personalizada del método
        super.onBatchScanResults(results);
        System.out.println("Batch scan results: (python) " + results);
        // System.out.println("Método onBatchScanResults");
    }

    public List<BluetoothDevice> getScanResults() {
        // Método que devuelve cada dispositivo escaneado
        System.out.println("getScanResults (python)" + this.scanResults);
        return this.scanResults;
    }

    public int getErrorCode() {
        // Método que devuelve el código de error
        System.out.println("getErrorCode (python)");
        return this.errorCode;
    }
}