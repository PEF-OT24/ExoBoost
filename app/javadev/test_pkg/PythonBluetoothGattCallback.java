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
    public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
        super.onConnectionStateChange(gatt, status, newState);
        System.out.println("onConnectionStateChange (python)");
        System.out.println("Status: " + status);
        System.out.println("New state: " + newState);
        this.connected_gatt = gatt;
    }

    public BluetoothGatt getConnectedGatt() {
        return this.connected_gatt;
    }
}
