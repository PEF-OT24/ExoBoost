from jnius import autoclass, PythonJavaClass, java_method, JavaClass, MetaJavaClass
from android.permissions import request_permissions, Permission # type: ignore
from time import sleep
import os

os.environ['CLASSPATH'] = 'javadev'

# Se importan las clases de Android java con Python for Android mediante pyjnius
Context = autoclass('android.content.Context')
PythonActivity = autoclass('org.kivy.android.PythonActivity').mActivity

BluetoothLeScanner = autoclass('android.bluetooth.le.BluetoothLeScanner')
BluetoothAdapter = autoclass('android.bluetooth.BluetoothAdapter') # Dispositivo actual
BluetoothManager = autoclass('android.bluetooth.BluetoothManager')
BluetoothDevice = autoclass('android.bluetooth.BluetoothDevice') # Dispositvos encontrados

PythonScanCallback = autoclass('javadev.test_pkg.PythonScanCallback') # Callback al realizar escaneo
ScanResult = autoclass('android.bluetooth.le.ScanResult') # Resultado

class BluetoothManager_App:
    '''Clase principal para el manejo de Bluetooth'''
    def __init__(self):
        # ----------- Métodos inicializadores -----------
        self.request_ble_permissions() # Solicitar permisos
        self.scanning: bool = False

        # ----------- Atributos de BLE -----------
        # Entorno de Python para Android
        self.context = PythonActivity

        # Manejo de BLE principal
        self.bluetooth_adapter = self.initialize_bluetooth()

        # Escaneador de BLE
        self.ble_enable = self.is_bluetooth_enabled()
        if self.ble_enable == None: self.ble_scanner = None
        else: self.ble_scanner = self.bluetooth_adapter.getBluetoothLeScanner()

        print("Creando objecto de ScanCallback")
        self.python_scan_callback = PythonScanCallback() # Se obtiene la instanciad del ScanCallback

        # ----------- Atributos lógicos -----------
        self.found_devices = [] # Arreglo para guardar dispositivos

    def initialize_bluetooth(self):
        '''Inicializa el objeto BluetoothAdapter'''
        bluetooth_manager = PythonActivity.getSystemService(Context.BLUETOOTH_SERVICE)
        bluetooth_adapter = bluetooth_manager.getAdapter()
        return bluetooth_adapter

    def request_ble_permissions(self):
        '''Solicitar permisos para el uso de Bluetooth'''
        permissions = [
            Permission.BLUETOOTH, 
            Permission.BLUETOOTH_ADMIN, 
            Permission.BLUETOOTH_CONNECT, 
            Permission.BLUETOOTH_SCAN, 
            Permission.ACCESS_FINE_LOCATION,
            Permission.ACCESS_COARSE_LOCATION
        ]
        request_permissions(permissions)

    def is_bluetooth_enabled(self) -> bool:
        '''Detecta si el BLE está habilitado'''
        estado = self.bluetooth_adapter.isEnabled()
        print(estado)
        return estado

    def enable_bluetooth(self):
        '''Habilita el Bluetooth si no lo esta'''
        if not self.ble_enable:
            self.bluetooth_adapter.enable()
        self.ble_scanner = self.bluetooth_adapter.getBluetoothLeScanner()

    def start_ble_scan(self):
        '''
        Método para iniciar la búsqueda de disposiuivos. 
        Solo funciona cuando self.ble_scanner no es None
        '''
        if self.ble_scanner:
            print("Scanning for devices...")
            if self.bluetooth_adapter.isDiscovering():
                print("Cancelling discovery")
                self.ble_scanner.stopScan(self.python_scan_callback)
                self.bluetooth_adapter.cancelDiscovery()
            print("Scanning between devices...")
            # self.found_devices.clear() # Limpiar los dispositivos encontrados
            self.ble_scanner.startScan(self.python_scan_callback)
        else: 
            print("Error: Bluetooth no disponible")

    def stop_ble_scan(self):
        '''Detiene la búsqueda de dispositivos si el Bluetooth está habilitado y si estaba previamente buscando'''
        if self.ble_scanner and self.bluetooth_adapter.isDiscovering():
            print("Stopping scan")
            print(self.bluetooth_adapter.isDiscovering()) # Se detecta si el BLE está escaneando
            self.ble_scanner.stopScan(self.python_scan_callback)
            self.found_devices = self.python_scan_callback.getScanResults()

    def get_found_devices(self):
        '''Devuelve una lista de tuplas (nombre, direccion) de los dispositivos encontrados'''
        return self.found_devices