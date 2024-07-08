from jnius import autoclass, PythonJavaClass
from android.permissions import request_permissions, Permission # type: ignore
from time import sleep

# Se acceden a los permisos necesarios
permissions = [Permission.BLUETOOTH, Permission.BLUETOOTH_ADMIN, Permission.BLUETOOTH_CONNECT, Permission.BLUETOOTH_SCAN, Permission.ACCESS_FINE_LOCATION]

# Se importan las clases de Android java con Python for Android mediante pyjnius
BluetoothAdapter = autoclass('android.bluetooth.BluetoothAdapter')
BluetoothDevice = autoclass('android.bluetooth.BluetoothDevice')
PythonActivity = autoclass('org.kivy.android.PythonActivity')
BluetoothLeScanner = autoclass('android.bluetooth.le.BluetoothLeScanner')
ScanCallback = autoclass('android.bluetooth.le.ScanCallback')
ScanResult = autoclass('android.bluetooth.le.ScanResult')

# class ScanCallbackClass(ScanCallback):
#     def __init__(self, manager):
#         super(ScanCallbackClass, self).__init__()
#         self.manager = manager
#         self.errors = {
#             "0x00000001": "SCAN_FAILED_ALREADY_STARTED",
#             "0x00000002": "SACAN_FAILED_APPLICATION_REGISTRATION_FAILED",
#             "0x00000003": "SCAN_FAILED_FEATURE_UNSUPPORTED",
#             "0x00000004": "SCAN_FAILED_INTERNAL_ERROR",
#             "0x00000005": "SCAN_FAILED_OUT_OF_HARDWARE_RESOURCES",
#             "0x00000006": "SCAN_FAILED_SCANNING_TOO_FREQUENTLY",
#         }

#     def onScanResult(self, callbackType, result):
#         device = result.getDevice()
#         device_name = device.getName()
#         device_address = device.getAddress()
#         self.manager.found_devices.append((device_name, device_address))

#     def onBatchScanResults(self, results):
#         for result in results:
#             self.onScanResult(None, result)

#     def onScanFailed(self, errorCode):
#         print("Scan failed with error code:", errorCode)
#         try: 
#             print(self.errors[str(errorCode)])
#         except Exception as e: 
#             print("Error en el key del diccionario")
#             print(e)

class BluetoothManager:
    '''Clase principal para el manejo de Bluetooth'''
    def __init__(self):
        # Solicitar permisos
        request_permissions(permissions)

        # ----------- Atributos de BLE -----------
        # Entorno de Python para Android
        self.context = PythonActivity.mActivity

        # Manejo de BLE principal
        self.bluetooth_adapter = BluetoothAdapter.getDefaultAdapter()

        # Escaneador de BLE
        self.ble_enable = self.is_bluetooth_enabled()
        if self.ble_enable == None: self.ble_scanner = None
        else: self.ble_scanner = self.bluetooth_adapter.getBluetoothLeScanner()

        self.scan_callback = ScanCallback

        # ----------- Atributos lógicos -----------
        self.found_devices = [] # Arreglo para guardar dispositivos

        # ----------- Métodos inicializadores -----------
    
    def is_bluetooth_enabled(self) -> bool:
        '''Detecta si el BLE está habilitado'''
        return self.bluetooth_adapter.isEnabled()

    def enable_bluetooth(self):
        '''Habilita el Bluetooth si no lo esta'''
        if not self.ble_enable:
            self.bluetooth_adapter.enable()
            if self.ble_scanner == None:
                self.ble_scanner = self.bluetooth_adapter.getBluetoothLeScanner()

    def start_ble_scan(self):
        '''
        Método para iniciar la búsqueda de disposiuivos. 
        Solo funciona cuando self.ble_scanner no es None
        '''
        if self.ble_scanner:
            if self.bluetooth_adapter.isDiscovering():
                self.bluetooth_adapter.cancelDiscovery()
            self.found_devices.clear() # Limpiar los dispositivos encontrados
            self.ble_scanner.startScan(self.scan_callback)
        else: 
            print("Error: Bluetooth no disponible")

    def stop_ble_scan(self):
        '''Detiene la búsqueda de dispositivos si el Bluetooth está habilitado y si estaba previamente buscando'''
        if self.ble_scanner and self.bluetooth_adapter.isDiscovering():
            self.ble_scanner.stopScan(self.scan_callback)

    def get_found_devices(self):
        '''Devuelve una lista de tuplas (nombre, direccion) de los dispositivos encontrados'''
        return self.found_devices

def main():
    # Ejemplo de uso para implementar el manejo de Bluetooth
    bt_manager = BluetoothManager()

    # Habilita el Bluetooth
    bt_manager.enable_bluetooth()
    print("Bluetooth enabled:", bt_manager.is_bluetooth_enabled())

    # Inicia el escaneo de dispositivos
    bt_manager.start_ble_scan()
    print("Scanning for devices...")

    # Escanea por 10 segundos
    sleep(10)  

    # Detiene el escaneo
    bt_manager.stop_ble_scan()

    # Muestra los dispositivos encontrados
    devices = bt_manager.get_found_devices()
    print("Found devices:", devices)

if __name__ == "__main__":
    main()