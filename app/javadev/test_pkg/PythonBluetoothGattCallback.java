package javadev.test_pkg;

import java.net.ConnectException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutionException;
import java.util.HashMap;
import java.util.UUID;

import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothProfile;

public final class PythonBluetoothGattCallback extends BluetoothGattCallback {

    public BluetoothGatt connected_gatt = null;

    public PythonBluetoothGattCallback() {
        super();
        System.out.println("Objeto de BluetoothGattCallback creado en java (python)");
    }

    @Override
    public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, byte[] value) {
        super.onCharacteristicChanged(gatt, characteristic, value);
        System.out.println("onCharacteristicChanged (python)");
    }

    @Override
    public void onCharacteristicRead(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, byte[] value,
            int status) {
        super.onCharacteristicRead(gatt, characteristic, value, status);
        System.out.println("onCharacteristicRead (python)");
    }

    @Override
    public void onCharacteristicWrite(BluetoothGatt gatt,
            BluetoothGattCharacteristic characteristic,
            int status) {
        super.onCharacteristicWrite(gatt, characteristic, status);
        System.out.println("onCharacteristicWrite (python)");
    }

    @Override
    public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
        super.onConnectionStateChange(gatt, status, newState);
        System.out.println("onConnectionStateChange (python)");
        System.out.println("Status: " + status);
        System.out.println("New state: " + newState);
        this.connected_gatt = gatt;
    }

    @Override
    public void onDescriptorRead(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status, byte[] value) {
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
    public void onServicesChanged(BluetoothGatt gatt) {
        super.onServicesChanged(gatt);
        System.out.println("onServicesChanged (python)");
    }

    @Override
    public void onServicesDiscovered(BluetoothGatt gatt, int status) {
        super.onServicesDiscovered(gatt, status);
        System.out.println("onServicesDiscovered (python)");
    }

    public BluetoothGatt getConnectedGatt() {
        return this.connected_gatt;
    }
}
