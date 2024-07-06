from jnius import autoclass
BluetoothAdapter = autoclass('android.bluetooth.BluetoothAdapter')

bluetooth_adapter = BluetoothAdapter.getDefaultAdapter()
if bluetooth_adapter is None: 
    print("Bluetooth not supported on this device")
else:
    print("Bluetooth supported")
