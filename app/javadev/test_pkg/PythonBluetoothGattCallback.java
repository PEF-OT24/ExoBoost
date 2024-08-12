package javadev.test_pkg;

import java.net.ConnectException;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutionException;
import java.util.HashMap;
import java.util.Map;
import java.util.UUID;

import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothProfile;

public final class PythonBluetoothGattCallback extends BluetoothGattCallback {

    public BluetoothGatt connected_gatt = null;
    public String final_value = ""; // Se inicializa la variable de lectura de caracteristicas
    public boolean ready_to_read = false;
    public boolean show_info = true;

    private Map<String, Map<String, String>> read_values = new HashMap<>(); // Hashmap de valores de características
                                                                            // según sus UUIDs

    public PythonBluetoothGattCallback() {
        super();
        System.out.println("Objeto de BluetoothGattCallback creado en java (python)");
    }

    @Override
    public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) {
        super.onCharacteristicChanged(gatt, characteristic);
        System.out.println("onCharacteristicChanged (python)");
    }

    @Override
    public void onCharacteristicRead(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status) {
        super.onCharacteristicRead(gatt, characteristic, status);
        System.out.println("onCharacteristicRead (python)");
        System.out.println("Status (python):" + status); // Muestra el estatus de la lectura

        // Impresión del valor leído a string
        this.final_value = bytesToString(characteristic.getValue());
        System.out.println("Valor interpretado (python): " + final_value);

        this.ready_to_read = true; // Actualiza el estado de listo para lectura
    }

    @Override
    public void onCharacteristicWrite(BluetoothGatt gatt,
            BluetoothGattCharacteristic characteristic,
            int status) {
        super.onCharacteristicWrite(gatt, characteristic, status);
        System.out.println("onCharacteristicWrite (python)");
        System.out.println("Status (python):" + status); // Muestra el status de la escritura
    }

    @Override
    public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
        super.onConnectionStateChange(gatt, status, newState);
        System.out.println("onConnectionStateChange (python)");
        System.out.println("Status (python):" + status);
        System.out.println("New state (python): " + newState);
        this.connected_gatt = gatt;
    }

    @Override
    public void onDescriptorRead(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status) {
        super.onDescriptorRead(gatt, descriptor, status);
        System.out.println("onDescriptorRead (python)");
    }

    @Override
    public void onDescriptorWrite(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status) {
        super.onDescriptorWrite(gatt, descriptor, status);
        System.out.println("onDescriptorWrite (python)");
    }

    @Override
    public void onMtuChanged(BluetoothGatt gatt, int mtu, int status) {
        super.onMtuChanged(gatt, mtu, status);
        System.out.println("onMtuChanged (python)");
        System.out.println("Mtu (python): " + mtu);
    }

    @Override
    public void onPhyRead(BluetoothGatt gatt, int txPhy, int rxPhy, int status) {
        super.onPhyRead(gatt, txPhy, rxPhy, status);
        System.out.println("onPhyRead (python)");
    }

    @Override
    public void onPhyUpdate(BluetoothGatt gatt, int txPhy, int rxPhy, int status) {
        super.onPhyUpdate(gatt, txPhy, rxPhy, status);
        System.out.println("onPhyUpdate (python)");
    }

    @Override
    public void onReadRemoteRssi(BluetoothGatt gatt, int rssi, int status) {
        super.onReadRemoteRssi(gatt, rssi, status);
        System.out.println("onReadRemoteRssi (python)");
    }

    @Override
    public void onReliableWriteCompleted(BluetoothGatt gatt, int status) {
        super.onReliableWriteCompleted(gatt, status);
        System.out.println("onReliableWriteCompleted (python)");
    }

    @Override
    public void onServiceChanged(BluetoothGatt gatt) {
        super.onServiceChanged(gatt);
        System.out.println("onServiceChanged (python)");
    }

    @Override
    public void onServicesDiscovered(BluetoothGatt gatt, int status) {
        super.onServicesDiscovered(gatt, status);
        System.out.println("onServicesDiscovered (python)");
        System.out.println("Status (python):" + status); // Si imprime un 0 es acción completada exitosamente
    }

    // -------------------- Métodos personalizados de la clase de callback
    // --------------------
    public BluetoothGatt getConnectedGatt() { // Método para obtener el objeto gatt
        return this.connected_gatt;
    }

    public String bytesToString(byte[] bytes) { // Método para convertir un array de bytes a una cadena de texto
        if (bytes == null) {
            return null;
        }
        return new String(bytes, StandardCharsets.UTF_8); // Puedes cambiar la codificación si es necesario
    }

    public void putValue(String outerKey, String innerKey, String value) {
        // Método para añadir o modificar elementos en el diccionario
        this.read_values.computeIfAbsent(outerKey, k -> new HashMap<>()).put(innerKey, value);
    }

    public String getCharValue() {
        // Obtiene el valor de la característica leída
        String retornar = this.final_value;
        this.final_value = "";
        System.out.println("Valor pasado a python (python): " + retornar);
        return retornar;
    }

    public boolean isReady_to_read() {
        // Método para obtener el estado para una lectura lista
        if (this.show_info) {
            System.out.println("Listo para lectura (python): " + this.ready_to_read);
        }
        this.show_info = false;
        return this.ready_to_read;
    }

    public void characteristicRead() {
        // Método para reinicar la lectura
        this.ready_to_read = false;
    }
}
