# Ejemplo simple para detectar si el BLE esta disponible en el dispositivo
from jnius import autoclass

BluetoothAdapter = autoclass('android.bluetooth.BluetoothAdapter')
ScanResult = autoclass('android.bluetooth.le.ScanResult')

bluetooth_adapter = BluetoothAdapter.getDefaultAdapter()
if bluetooth_adapter is None: 
    print("Bluetooth not supported on this device")
else:
    print("Bluetooth supported")
