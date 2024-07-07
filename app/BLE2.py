from jnius import autoclass, cast
from android.permissions import request_permissions, Permission # type: ignore
from time import sleep

# Se acceden a los permisos necesarios
permissions = [Permission.BLUETOOTH, Permission.BLUETOOTH_ADMIN, Permission.ACCESS_FINE_LOCATION]
request_permissions(permissions)

# Se importan las clases de Android java con Python for Android mediante pyjnius
BluetoothAdapter = autoclass('android.bluetooth.BluetoothAdapter')
BluetoothDevice = autoclass('android.bluetooth.BluetoothDevice')
#IntentFilter = autoclass('android.content.IntentFilter')
#BroadcastReceiver = autoclass('android.content.BroadcastReceiver')
#Context = autoclass('android.content.Context')

"""
class DeviceReceiver(BroadcastReceiver):
    '''Clase para el manejo de los dispositivos encontrados'''
    def __init__(self, manager):
        self.manager = manager
        super(DeviceReceiver, self).__init__()

    def onReceive(self, context, intent):
        action = intent.getAction()
        if action == BluetoothDevice.ACTION_FOUND:
            device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
            device_name = device.getName()
            device_address = device.getAddress()
            self.manager.found_devices.append((device_name, device_address))

class BluetoothManager:
    '''Clase principal para el manejo de Bluetooth'''
    def __init__(self):

        # Crea el objeto principal para manejar el Bluetooth
        self.bluetooth_adapter = BluetoothAdapter.getDefaultAdapter()
        self.found_devices = [] # Arreglo para guardar dispositivos
        self.receiver = DeviceReceiver(self)
        self.intent_filter = IntentFilter(BluetoothDevice.ACTION_FOUND)
        self.context = autoclass('org.kivy.android.PythonActivity').mActivity

        # Indicador de si el Bluetooth esta habilitado
        self.ble_enable = self.is_bluetooth_enabled()

    def is_bluetooth_enabled(self) -> bool:
        '''Detecta si el BLE está habilitado'''
        return self.bluetooth_adapter.isEnabled()

    def enable_bluetooth(self):
        '''Habilita el Bluetooth si no lo esta'''
        if not self.ble_enable:
            self.bluetooth_adapter.enable()

    def start_discovery(self):
        '''Inicia la busqueda de dispositivos Bluetooth cancelando la anterior'''
        if self.bluetooth_adapter.isDiscovering():
            self.bluetooth_adapter.cancelDiscovery()
            sleep(0.1) # Espera 100 ms para volver a escanear
        self.found_devices = []
        self.context.registerReceiver(self.receiver, self.intent_filter)
        self.bluetooth_adapter.startDiscovery()

    def stop_discovery(self):
        '''Detiene la busqueda de dispositivos Bluetooth'''
        if self.bluetooth_adapter.isDiscovering():
            self.bluetooth_adapter.cancelDiscovery()
        self.context.unregisterReceiver(self.receiver)

    def get_found_devices(self):
        return self.found_devices

def main():
    bt_manager = BluetoothManager()

    # Habilita el Bluetooth
    bt_manager.enable_bluetooth()
    print("Bluetooth enabled:", bt_manager.is_bluetooth_enabled())

    # Inicia el escaneo de dispositivos
    bt_manager.start_discovery()
    print("Scanning for devices...")

    # Escanea por 10 segundos
    sleep(10)  

    # Detiene el escaneo
    bt_manager.stop_discovery()

    # Muestra los dispositivos encontrados
    devices = bt_manager.get_found_devices()
    print("Found devices:", devices)

if __name__ == "__main__":
    main()
"""