package javadev.test_pkg;

// Importar clases necesarias de java
import java.net.ConnectException;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutionException;
import java.util.HashMap;
import java.util.Map;
import java.util.UUID;
import java.util.List;
import java.util.ArrayList;

// Importar clasese necesarias de android
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.BluetoothGattService;

public final class PythonBluetoothGattCallback extends BluetoothGattCallback {

    public BluetoothGatt connected_gatt = null;
    public boolean ready_to_read = false;
    public boolean show_info = true;

    // UUID del servicio y la característica a leer
    public String serviceToRead = "";
    public String characteristicToRead = "";

    // UUID del servicio y la característica a escribir
    public String serviceToWrite = "";
    public String characteristicToWrite = "";

    // UUID del servicio y la característica notificada para leer
    public String serviceNotified = "";
    public String characteristicNotified = "";
    public boolean ReadIndicated = false;

    // Bandera de notificaciones habilitada exitosamente
    public boolean notification_enabled = false;

    private Map<String, Map<String, String>> read_values = new HashMap<>(); // Hashmap de valores de características
                                                                            // según sus UUIDs

    public PythonBluetoothGattCallback() {
        super();
        System.out.println("Objeto de BluetoothGattCallback creado en java (python)");
    }

    @Override
    public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) {
        // Método que se ejecuta cada que se recibe una notificación.

        super.onCharacteristicChanged(gatt, characteristic);
        System.out.println("onCharacteristicChanged (python)");

        // Reconocimiento de UUID de servicio y característica de la notificación
        this.characteristicNotified = characteristic.getUuid().toString();
        List<BluetoothGattService> services = gatt.getServices();

        // Se recorren los servicios para encontrar el correspondiente
        for (BluetoothGattService servicio : services) {
            String UUID_servicio = servicio.getUuid().toString();
            System.out.println("UUID_servicio (python): " + UUID_servicio);

            // Para cada servicio se descubren características
            List<BluetoothGattCharacteristic> characteristics = servicio.getCharacteristics();
            for (BluetoothGattCharacteristic characteristic_for : characteristics) {
                String UUID_caracteristica = characteristic_for.getUuid().toString();
                System.out.println("UUID_caracteristica (python): " + UUID_caracteristica);
                if (UUID_caracteristica == this.characteristicNotified) {
                    System.out.println("Servicio correspondiente encontrado (python): " + UUID_servicio);
                    this.serviceNotified = UUID_servicio;
                    break;
                }
            }
        }

        System.out.println(this.serviceNotified);
        System.out.println("Notificación recibida: (python): "
                + this.serviceNotified + "::" + this.characteristicNotified);
        this.ReadIndicated = true; // Se indica que se debe leer una característica
    }

    @Override
    public void onCharacteristicRead(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status) {
        super.onCharacteristicRead(gatt, characteristic, status);

        System.out.println("onCharacteristicRead (python)");
        System.out.println("Status (python):" + status); // Muestra el estatus de la lectura

        // Se guarda el valor leído
        String read_value = bytesToString(characteristic.getValue());
        this.putValue(this.serviceToRead, this.characteristicToRead, read_value);

        this.ready_to_read = true; // Actualiza el estado de listo para lectura
    }

    @Override
    public void onCharacteristicWrite(BluetoothGatt gatt,
            BluetoothGattCharacteristic characteristic,
            int status) {
        super.onCharacteristicWrite(gatt, characteristic, status);
        System.out.println("onCharacteristicWrite (python)");
        System.out.println("Status (python):" + status); // Muestra el status de la escritura

        // Se guarda el valor escrito
        String read_value = bytesToString(characteristic.getValue());
        this.putValue(this.serviceToWrite, this.characteristicToWrite, read_value);
    }

    @Override
    public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
        super.onConnectionStateChange(gatt, status, newState);
        System.out.println("onConnectionStateChange (python)");
        System.out.println("Status (python):" + status);
        System.out.println("New state (python): " + newState);
        if (newState == 2) { // Se conectó
            this.connected_gatt = gatt;
            System.out.println("Conectado (python)");

            // Se descubren los servicios
            this.connected_gatt.discoverServices();
            List<BluetoothGattService> services = this.connected_gatt.getServices();

            // Se recorren los elementos para establecer valores iniciales
            for (BluetoothGattService servicio : services) {
                String UUID_servicio = servicio.getUuid().toString();
                // Para cada servicio se descubren características
                List<BluetoothGattCharacteristic> characteristics = servicio.getCharacteristics();
                for (BluetoothGattCharacteristic characteristic : characteristics) {
                    String UUID_caracteristica = characteristic.getUuid().toString();
                    // Se inicializan los valores como un "null"
                    this.putValue(UUID_servicio, UUID_caracteristica, "null");
                }
            }

        } else if (newState == 0) { // Se desconectó
            this.connected_gatt = null;
            System.out.println("Desconectado (python)");
        }
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

        // Verifica si se activó la notificación con éxito
        if (status == BluetoothGatt.GATT_SUCCESS) { // GATT_SUCESS = 0
            this.notification_enabled = true;
            System.out.println("Notificación activada (python)");
        }
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

    // ------------ Métodos personalizados de la clase de callback ------------
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

    public String getValue(String outerKey, String innerKey) {
        // Método para obtener el valor de una característica específica
        Map<String, String> innerMap = this.read_values.get(outerKey);
        if (innerMap != null) {
            return innerMap.get(innerKey);
        }
        return "null"; // Retorna null si la clave externa o interna no existe
    }

    public boolean isReady_to_read() {
        // Método para obtener el estado para una lectura lista
        if (this.show_info) {
            System.out.println("Listo para lectura (python): " + this.ready_to_read);
        }
        this.show_info = false;
        return this.ready_to_read;
    }

    public void reset_reading() {
        // Método para reinicar la lectura
        this.ready_to_read = false;
    }

    public void CharToRead(String serviceUUID, String characteristicUUID) {
        // Método para definir el UUID del servicio a leer
        this.serviceToRead = serviceUUID;
        this.characteristicToRead = characteristicUUID;
    }

    public void CharToWrite(String serviceUUID, String characteristicUUID) {
        // Método para definir el UUID del servicio a leer
        this.serviceToWrite = serviceUUID;
        this.characteristicToWrite = characteristicUUID;
    }

    public void ReadFlag() {
        // Método que resetea la bandera de lectura
        if (this.ReadIndicated) {
            this.ReadIndicated = false;
        }
    }

    public boolean notification_flag() {
        // Método para obtener el estado de la notificación (exitosa no exitosa)
        return this.notification_enabled;
    }
}