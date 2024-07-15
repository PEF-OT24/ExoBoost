from jnius import autoclass, PythonJavaClass, java_method, JavaClass, MetaJavaClass
from android.permissions import request_permissions, Permission # type: ignore
from time import sleep
from threading import Thread, Timer, Event
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
        '''Inicializa el objeto BluetoothAdapter automáticamente'''
        # ----------- Métodos inicializadores -----------
        self.request_ble_permissions() # Solicitar permisos

        # ----------- Atributos -----------
        # Entorno de Python para Android
        self.context = PythonActivity
        self.scanning: bool = False
        self.connected: bool = False

        # Manejo de BLE principal
        self.bluetooth_adapter = self.initialize_bluetooth()

        # Escaneador de BLE
        self.ble_enable = self.is_bluetooth_enabled()
        if not(self.ble_enable): self.enable_bluetooth()

        # Se crea el escaneador de BLE
        self.ble_scanner = self.bluetooth_adapter.getBluetoothLeScanner()
        self.python_scan_callback = PythonScanCallback() # Se obtiene la instanciad del ScanCallback

        # ----------- Atributos lógicos -----------
        self.found_devices = [] # Arreglo para guardar dispositivos
        self.scanning_event = Event() # Evento para manejar

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
        '''Habilita el Bluetooth'''
        self.bluetooth_adapter.enable()

    def start_ble_scan(self):
        '''
        Método para iniciar la búsqueda de disposiuivos. 
        Solo funciona cuando self.ble_scanner no es None
        '''
        try:
            print("Scanning between devices...")
            self.ble_scanner.startScan(self.python_scan_callback)
        except Exception as e:
            print(f"Error: Bluetooth no disponible: {e}")

    def stop_ble_scan(self):
        '''Detiene la búsqueda de dispositivos si el Bluetooth está habilitado y si estaba previamente buscando'''
        if self.ble_scanner:
            print("Stopping scan")
            self.ble_scanner.stopScan(self.python_scan_callback)
            return self.python_scan_callback.getScanResults()