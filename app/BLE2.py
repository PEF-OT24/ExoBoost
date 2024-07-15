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
        self.found_devices: list[BluetoothDevice] = [] # type: ignore # Arreglo para guardar dispositivos

        # Manejo de BLE principal
        self.bluetooth_adapter = self.initialize_bluetooth()

        # Escaneador de BLE
        self.ble_enable = self.is_bluetooth_enabled()
        if not(self.ble_enable): self.enable_bluetooth()

        # Se crea el escaneador de BLE
        self.ble_scanner = self.bluetooth_adapter.getBluetoothLeScanner()
        self.python_scan_callback = PythonScanCallback() # Se obtiene la instanciad del ScanCallback

        # ----------- Atributos lógicos -----------
        self.scanning: bool = False
        self.connected: bool = False

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
        Método para iniciar la búsqueda de dipositivos. 
        Solo funciona cuando no está conectado ni escaneando
        '''
        try:
            if not(self.scanning) and not(self.connected):
                self.scanning = True
                print("Scanning between devices...")
                self.ble_scanner.startScan(self.python_scan_callback)
            else: print("Already scanning")
        except Exception as e:
            print(f"Error: Bluetooth no disponible: {e}")

    def stop_ble_scan(self) -> list[str]:
        '''Detiene la búsqueda de dispositivos si el Bluetooth está habilitado y si estaba previamente buscando'''
        if self.ble_scanner and self.scanning:
            print("Stopping scan")
            self.ble_scanner.stopScan(self.python_scan_callback)
            self.scanning = False

            # Se obtienen los dispositivos
            print("Getting found devices")
            self.found_devices.clear()
            self.found_devices = self.python_scan_callback.getScanResults()

            # Se obtienen los nombres de los dispositivos y se retornan
            nombres = [name for name in self.found_devices.getName()]
            return nombres

    def connect_disconnect(self, device_name: str) -> bool:
        '''
        Se conecta al dispositivo indicado por su nombre
        Entrada: device_name str -> Nombre del dispositivo
        Salida: True si se conectó correctamente, False de lo contrario
        '''
        try: # Intenta conectarse
            if not(self.connected) and self.found_devices:
                print("Connecting to device...")

                # Se busca al dispositivo 
                for device in self.found_devices:
                    if device.getName() == device_name:
                        target_device = device
                        return True
                    
                # Intento de conexión
            else: print("Already connected")
        except Exception as e:
            print(f"Error de Bluetooth: {e}")
            return False