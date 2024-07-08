from jnius import autoclass, cast
from android.permissions import request_permissions, Permission # type: ignore
import time

# Request necessary permissions
permissions = [Permission.BLUETOOTH, Permission.BLUETOOTH_ADMIN, Permission.ACCESS_FINE_LOCATION]
request_permissions(permissions)

# Get necessary classes from Android
BluetoothAdapter = autoclass('android.bluetooth.BluetoothAdapter')
BluetoothDevice = autoclass('android.bluetooth.BluetoothDevice')
IntentFilter = autoclass('android.content.IntentFilter')
BroadcastReceiver = autoclass('android.content.BroadcastReceiver')
PythonActivity = autoclass('org.kivy.android.PythonActivity')

class BluetoothManager:
    def __init__(self):
        self.bluetooth_adapter = BluetoothAdapter.getDefaultAdapter()
        self.found_devices = []
        self.context = PythonActivity.mActivity

        # Create a receiver as a simple function
        self.receiver = self.create_receiver()
        self.intent_filter = IntentFilter(BluetoothDevice.ACTION_FOUND)

    def create_receiver(self):
        # Define a BroadcastReceiver with a simple function
        def onReceive(context, intent):
            action = intent.getAction()
            if action == BluetoothDevice.ACTION_FOUND:
                device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
                device_name = device.getName()
                device_address = device.getAddress()
                self.found_devices.append((device_name, device_address))

        return BroadcastReceiver(onReceive)

    def is_bluetooth_enabled(self):
        return self.bluetooth_adapter.isEnabled()

    def enable_bluetooth(self):
        if not self.bluetooth_adapter.isEnabled():
            self.bluetooth_adapter.enable()

    def start_discovery(self):
        if self.bluetooth_adapter.isDiscovering():
            self.bluetooth_adapter.cancelDiscovery()
        self.found_devices = []
        self.context.registerReceiver(self.receiver, self.intent_filter)
        self.bluetooth_adapter.startDiscovery()

    def stop_discovery(self):
        if self.bluetooth_adapter.isDiscovering():
            self.bluetooth_adapter.cancelDiscovery()
        self.context.unregisterReceiver(self.receiver)

    def get_found_devices(self):
        return self.found_devices

def main():
    bt_manager = BluetoothManager()

    # Enable Bluetooth
    bt_manager.enable_bluetooth()
    print("Bluetooth enabled:", bt_manager.is_bluetooth_enabled())

    # Scan for devices
    bt_manager.scan_for_devices()
    print("Scanning for devices...")

    # Wait for a while to discover devices
    time.sleep(10)  # Adjust the sleep time as necessary

    # Stop discovery
    bt_manager.stop_discovery()

    # Get found devices
    devices = bt_manager.get_found_devices()
    print("Found devices:", devices)

if __name__ == "__main__":
    main()
