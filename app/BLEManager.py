'''
Módulo de manejo de BLE para Android desde P4A. 
Este módulo tiene dependencias en pyjnius v. 1.6.1.
Se importan clases de Android SDK (API v. 18 a v. 32) en el paquete javadev/test_pkg. Se emplea las APIs de Android SDK v. 31.
Funciona para Android 12 (no apto para Android 13).

Este módulo es parte del Proyecto de Evaluación Final para la Carrera de Ingeniería en Mecatrónica en la UDEM. 

Fecha creada: 06/06/2024
Asesor: Dr. Mario Claros
Autores: Teresa Hernandez, Carlos Reyes y David Villanueva.

Ante cualquier duda contactar: david.villanueva@udem.edu
'''

from jnius import autoclass
from android.permissions import request_permissions, Permission # type: ignore
from time import sleep
import os
import json 

# Se establece la ruta de la librería para Android
os.environ['CLASSPATH'] = 'javadev'

# Base genérica para los UUID personalizados
BASE_UUID = "00000000-0000-1000-8000-00805f9b34fb"

# Se importan las clases de Android SDK (API v. 31) en el paquete con P4A
Context = autoclass('android.content.Context')
PythonActivity = autoclass('org.kivy.android.PythonActivity').mActivity

BluetoothLeScanner = autoclass('android.bluetooth.le.BluetoothLeScanner')
BluetoothAdapter = autoclass('android.bluetooth.BluetoothAdapter') # Dispositivo actual
BluetoothManager = autoclass('android.bluetooth.BluetoothManager')
BluetoothDevice = autoclass('android.bluetooth.BluetoothDevice') # Dispositvos encontrados
BluetoothGatt = autoclass('android.bluetooth.BluetoothGatt')     # GATT del dispositivo encontrado
BluetoothGattService = autoclass('android.bluetooth.BluetoothGattService') # Clase de los servicios descubiertos
BluetoothGattCharacteristic = autoclass('android.bluetooth.BluetoothGattCharacteristic') # Clase de las características de los servicios
BluetoothGattDescriptor = autoclass('android.bluetooth.BluetoothGattDescriptor') # Clase de los descriptores de características
ScanResult = autoclass('android.bluetooth.le.ScanResult') # Resultado
UUIDClass = autoclass('java.util.UUID') 

# Se importan las clases del paquete personalizado 
PythonScanCallback = autoclass('javadev.test_pkg.PythonScanCallback') # Callback al realizar escaneo
PythonBluetoothGattCallback = autoclass('javadev.test_pkg.PythonBluetoothGattCallback') # Callback al conectar

class Characteristic_Info:
    '''
    Clase que analiza la información sobre una característica dada. 
    Entrada: characteristic BluetoothGattCharacteristic -> Característica a analizar
    ''' 

    def __init__(self, characteristic: BluetoothGattCharacteristic) -> None: # type: ignore
        '''Método constructor de la clase'''
        self.characteristic = characteristic
        self.properties: int = characteristic.getProperties() # Se obtienen las properties de la característica

        # Posibles propiedades de la característica 
        self.all_properties = {
            "BROADCAST": BluetoothGattCharacteristic.PROPERTY_BROADCAST,
            "EXTENDED PROPS": BluetoothGattCharacteristic.PROPERTY_EXTENDED_PROPS,
            "INDICATE": BluetoothGattCharacteristic.PROPERTY_INDICATE,
            "NOTIFY": BluetoothGattCharacteristic.PROPERTY_NOTIFY,
            "READ": BluetoothGattCharacteristic.PROPERTY_READ,
            "SIGNED WRITE": BluetoothGattCharacteristic.PROPERTY_SIGNED_WRITE,
            "WRITE": BluetoothGattCharacteristic.PROPERTY_WRITE,
            "WRITE WITH NO RESPONSE": BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE,
        }

    def get_uuid(self) -> str: # type: ignore
        '''
        Método que obtiene el UUID de una característica en formato de string
        Entrada: characteristic BluetoothGattCharacteristic -> Característica
        Salida: str -> UUID de la característica en formato de string
        '''
        return self.characteristic.getUuid().toString()
    
    def contains_property(self, property: int) -> bool: # type: ignore
        '''Comprueba si una característica contiene una propiedad en espefícico
        Entradas: characteristic BluetoothGattCharacteristic -> Característica a analizar
                  property int -> Propiedad a buscar

        Salida: bool -> True si la característica contiene la propiedad indicada, False de lo contrario
        '''
        return True if self.properties & property == property else False
    
    def isBroadastable(self) -> bool: # type: ignore
        '''Comprueba si la característica es Broadcastable
        Salida: bool -> True si la característica es Broadcastable, False de lo contrario'''
        return self.contains_property(self.all_properties["BROADCAST"])
    
    def isIndicatable(self) -> bool: # type: ignore
        '''Comprueba si la característica es Indicatable
        Salida: bool -> True si la característica es Indicatable, False de lo contrario'''
        return self.contains_property(self.all_properties["INDICATE"])
    
    def isNotifiable(self) -> bool: # type: ignore
        '''Comprueba si la característica es Notifiable
        Salida: bool -> True si la característica es Notifiable, False de lo contrario'''
        return self.contains_property(self.all_properties["NOTIFY"])
    
    def isReadable(self) -> bool: # type: ignore
        '''Comprueba si la característica es Readable
        Salida: bool -> True si la característica es Readable, False de lo contrario'''
        return self.contains_property(self.all_properties["READ"])
    
    def isWriteable(self) -> bool: # type: ignore
        '''Comprueba si la característica es Writeable
        Salida: bool -> True si la característica es Writeable, False de lo contrario'''
        return self.contains_property(self.all_properties["WRITE"])
        

class BluetoothManager_App:
    '''Clase principal para el manejo de Bluetooth'''
    def __init__(self):
        '''Constructor de la clase'''
        # ----------- Métodos inicializadores -----------
        self.request_ble_permissions() # Solicitar permisos

        # ----------- Atributos -----------
        # Entorno de P4A
        self.context = PythonActivity
        self.found_devices: list[BluetoothDevice] = [] # type: ignore # Arreglo para guardar dispositivos

        # Manejo de BLE principal
        self.bluetooth_adapter = self.initialize_bluetooth()

        # Se habilita el Bluetooth en el dispositivo
        self.ble_enable = self.is_bluetooth_enabled()
        if not(self.ble_enable): self.enable_bluetooth()

        # Se crea el escaneador de BLE
        self.ble_scanner = self.bluetooth_adapter.getBluetoothLeScanner()
        self.python_scan_callback = PythonScanCallback()          # Instancia de Callback para escaneo
        self.python_gatt_callback = PythonBluetoothGattCallback() # Instancia de Callback para el estado del GATT

        # ----------- Atributos lógicos -----------
        self._GATT_MAX_MTU_SIZE = 517               # Tamaño máximo de transmisión
        self.scanning: bool = False                 # Bandera de escaneo
        self.connected: bool = False                # Bandera de conectado
        self.connected_device: BluetoothDevice = None # type: ignore              # Dispositivo conectado
        self.connected_gatt: BluetoothGatt = None # type: ignore                  # GATT del dispositivo conectado
        self.discovered_services: list[BluetoothGattService] = [] # type: ignore  # Lista de servicios descubiertos
        self.discovered_characteristics: dict[BluetoothGattCharacteristic] = {} # type: ignore # Diccionario de características descubiertas

    def initialize_bluetooth(self):
        '''Método que inicializa el objeto BluetoothAdapter'''
        bluetooth_manager = self.context.getSystemService(Context.BLUETOOTH_SERVICE)
        bluetooth_adapter = bluetooth_manager.getAdapter()
        return bluetooth_adapter
    
    def resetBLE(self): 
        '''Método que llama al método resetScanning de PythonScanCallback para reiniciar el escaneo'''
        self.python_scan_callback.resetScanning()

    def request_ble_permissions(self):
        '''Método para solicitar permisos para el uso de Bluetooth en Android'''
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
        '''Método que detecta si el BLE está habilitado'''
        return self.bluetooth_adapter.isEnabled()
        
    def enable_bluetooth(self):
        '''Método para habilitar el Bluetooth'''
        self.bluetooth_adapter.enable()

    def start_ble_scan(self):
        '''
        Método para iniciar la búsqueda de dipositivos. 
        El Bluetooth Adatparter no debe estar conectado ni escaneando
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
        '''Método que detiene la búsqueda de dispositivos si el Bluetooth está habilitado y si estaba previamente buscando
        Salida: devices_names list[str] -> Lista de nombres de los dispositivos encontrados'''
        if self.ble_scanner and self.scanning:
            self.ble_scanner.stopScan(self.python_scan_callback)
            self.scanning = False

            # Se obtienen los dispositivos
            self.found_devices.clear()
            self.found_devices = self.python_scan_callback.getScanResults()

            # Se obtienen los nombres de los dispositivos y se retornan
            devices_names = [name.getName() for name in self.found_devices]
            return devices_names

    def connect(self, device_name: str) -> bool:
        '''
        Es necesario detener el escaneo antes de realizar aciones de conexión; se llama al método stop_ble_scan().
        Se conecta  o desconecta al dispositivo indicado por su nombre. 
        Este método trabaja a la par con el callback self.python_gatt_callback que se encarga de la interacción con GATT
        Cuando se establece la conexión, se guarda el GATT del dispositivo conectado para manipular en otros métodos.

        Entrada: device_name str -> Nombre del dispositivo
        Salida: True si la acción se realizó correctamente, False de lo contrario
        '''
        # Se debe detener el escaneo para poder conectarse o desconectarse
        if self.scanning: return False

        try: # Intenta conectarse
            if not(self.connected):
                print("Connecting to device...")

                # Se busca al dispositivo 
                for device in self.found_devices:
                    if device.getName() == device_name:
                        target_device = device
                        break
                    
                # Se realiza la conexion
                print("Establishing connection...")
                target_device.connectGatt(self.context.getApplicationContext(), 
                                          False, 
                                          self.python_gatt_callback, 
                                          transport = BluetoothDevice.TRANSPORT_LE # Para testing
                                          )

                # Se espera 2 segundos para que el dispositivo se conecte
                print("Waiting for connection...")
                sleep(2)
                print(f"Bond state: {target_device.getBondState()}")

                # Se guarda el GATT y el dispositivo conectado
                print("Getting GATT...")
                self.connected_gatt = self.python_gatt_callback.getConnectedGatt()
                self.connected_device = target_device

                print("Establishing MTU...")
                self.connected_gatt.requestMtu(self._GATT_MAX_MTU_SIZE) # Se establece el tamaño máximo de la transmisión de datos 

                print("Getting device info...")

                # Se descubre la información del dispositivo y se muestran los UUIDs
                self.discover_services_and_characteristics()

                # Se marca como conectado
                self.connected = True

                self.show_uuids()
            else: print("Already connected") # IMPLEMENTAR MÉTODO PARA DESCONECTARSE
            return True
        except Exception as e:
            # En este punto los errores pueden ser: 
            # dispositivo no encontrado, escaneo en proceso, bluetooth no correctamente inicializado, dispositivo ya conectado, nombre mal escrito.
            print(f"Error de Bluetooth: {e}")
            return False
        
    def disconnect(self):
        '''Método para desconectar de un dispositivo ya conectado'''
        try: 
            if self.connected:
                self.connected_gatt.disconnect()
                # self.connected_gatt.close()

                # Se limpian los atributos después de desconectarse
                self.connected = False
                self.connected_gatt = None
                self.connected_device = None
                self.discovered_characteristics: dict = {}
                self.discovered_services: list = []
        except Exception as e:
            print(f"Error de Bluetooth al intentar desconectarse: {e}")

    def discover_services(self, wait_time: float) -> list[BluetoothGattService]: # type: ignore
        '''Método que descubre los servicios de un dispositivo ya conectado
        Entrada: wait_time float -> tiempo de espera para la descubrimiento de los servicios
        Salida: list[BluetoothGattService] -> Lista de los servicios encontrados
        '''
        if not self.connected_gatt: return # Marca error si no hay un GATT conectado

        # Llama al método para comenzar el descubrimiento de los servicios
        success_flag: bool = self.connected_gatt.discoverServices() # True si se descubrieron los servicios
        sleep(wait_time) # Espera 0.5 segundos para que se descubran los servicios

        # Si no ha terminado el escaneado devuelve False
        # LÓGICA INCORRECTA, LA BANDERA DEBE ACTUALIZARSE EN UN MÉTODO ASINCRÓNICO
        if not success_flag: 
            print("Failed to discover services") 
            return
        
        # Se obtienen los servicios
        discovered_services = self.connected_gatt.getServices()

        # Si no hay servicios, marca error
        if not discovered_services:
            print("No services found")
            return

        return discovered_services
    
    def discover_characteristics(self, service: BluetoothGattService) -> list[BluetoothGattCharacteristic]: # type: ignore
        '''
        Método que descubre las características de un servicio ya descubierto. El procedimiento toma algo de tiempo, por lo que se recomienda
        que se llame en un hilo separado.
        
        Entrada: service BluetoothGattService     -> Servicio cuyas características se descubren
        Salida: list[BluetoothGattCharacteristic] -> Lista de las características del servicio
        '''        
        # Se obtienen las características
        characteristics: list[BluetoothGattCharacteristic] = service.getCharacteristics() # type: ignore
        print(f"Discovered characteristics: {characteristics}")
        return list(characteristics) # Se devuelve en formato de lista 
    
    def discover_services_and_characteristics(self, wait_time: float = 0.5) -> None: # type: ignore
        '''
        Método que descubre los servicios y las características de un dispositivo ya conectado
        Los resultados se guardan en self.discovered_services y self.discovered_characteristics de la clase.

        Entrada: wait_time float -> tiempo de espera en segundos para descubrir los servicios, por defalut un valor de 0.5
        '''
        # Se descubren todos los servicios
        self.discovered_services = self.discover_services(wait_time)

        # Para cada servicio se descubren sus características y se guardan en el diccionario
        for service in self.discovered_services:
            service_uuid = service.getUuid()
            uuid_name = service_uuid.toString() # type: ignore
            print("Características del servicio (UUID): " + uuid_name)
            self.discovered_characteristics[uuid_name] = self.discover_characteristics(service)

    def show_uuids(self):
        '''Método que muestra los UUIDs de cada característica de cada servicio'''
        # Comprueba que hay un dispositivo conectado
        if not self.connected: raise Exception("Error al imprimir UUIDs: No hay un dispositivo conectado")

        # Se imprime el UUID de cada característica para cada serivicio
        for service_uuid in list(self.discovered_characteristics.keys()): 
            print(f"Servicio UUID: {service_uuid}")
            print(f"Características UUID's:")
            for i, car in enumerate(self.discovered_characteristics[service_uuid]):
                car_uuid = car.getUuid()
                print(f"Característica {i}: {car_uuid.toString()}") # Se imprime como string
            print("-------------")

    def write(self, service_uuid: str, characteristic_uuid: str, data: str) -> None: # type: ignore
        '''
        Método que escribe sobre la característica indicada de un servicio del dispositivo conectado

        Entradas: service_uuid str -> UUID del servicio de la característica
                  characteristic_uuid str -> UUID de la característica sobre la que se escribe
                  data str -> mensaje a enviar
        '''

        # Comprueba que hay un dispositivo conectado
        if not self.connected: 
            print("Dispositivo no conectado")
            return

        try: 
            # Identifica la característica de interés del servicio de interés con los UUID's
            for i, char in enumerate(self.discovered_characteristics[service_uuid]):
                if char.getUuid().toString() == characteristic_uuid: 
                    characteristic = char
                    index = i
                    break

            car_analyzer = Characteristic_Info(characteristic)
        
            # Continúa si se puede acceder a la característica
            if not (car_analyzer.isWriteable() and car_analyzer.isReadable()): raise Exception("Característica no accesible")

            # Se configura el envío del mensaje
            characteristic.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT) # Write request (espera respuesta de confirmación)
            characteristic.setValue(data) # Se establece el mensaje como String
            
            # Se escribe la característica
            self.python_gatt_callback.CharToWrite(service_uuid, characteristic_uuid)
            self.connected_gatt.writeCharacteristic(characteristic)

            # Se guarda la configuración de la característica en la lista original 
            self.discovered_characteristics[service_uuid][index] = characteristic

        except Exception as e:
            print("Característica no encontrada")
            print(f"Error: {e}")
    
    def write_json(self, service_uuid: str, characteristic_uuid: str, data): 
        '''Método que convierte el archivo JSON en un string y llama al método write()
        Entradas: service_uuid str -> UUID del servicio de la característica
                  characteristic_uuid str -> UUID de la característica
                  data str -> mensaje a enviar
        '''

        # Marca error si el archivo no es un diccionario
        if not isinstance(data, dict):  
            print("El archivo JSON no es un diccionario")
            return

        # Se llama al método write()
        self.write(service_uuid, characteristic_uuid, json.dumps(data))

    def read(self, service_uuid: str, characteristic_uuid: str) -> str:  # type: ignore 
        '''Método que lee el valor de una característica de un servicio específicos.
        Entradas: service_uuid str -> UUID del servicio de la característica a leer 
                  characteristic_uuid str -> UUID de la característica a leer
                  data str -> mensaje a enviar
        '''
        
        if not self.connected: 
            print("Error al leer: No hay un dispositivo conectado")
            return
        
        try: 
            # Identifica la característica de interés del servicio de interés con los UUID's
            for i, char in enumerate(self.discovered_characteristics[service_uuid]):
                if char.getUuid().toString() == characteristic_uuid: 
                    characteristic = char
                    print(f"Característica encontrada: {characteristic_uuid}")
                    index = i
                    break

            car_analyzer = Characteristic_Info(characteristic)
        
            # Continúa si se puede acceder a la característica
            if not car_analyzer.isReadable(): raise Exception("Característica no accesible")

            # Se configura la lectura del mensaje
            self.python_gatt_callback.CharToRead(service_uuid, characteristic_uuid)
            self.connected_gatt.readCharacteristic(characteristic) # Se lee la característica

            # Espera hasta que la característica se pueda leer
            self.python_gatt_callback.show_info = False # Mensaje no mostrado 
            while not(self.python_gatt_callback.isReady_to_read()): pass # Espera hasta que la lectura esté disponible
            self.python_gatt_callback.reset_reading()                    # Reinicia la bandera

            # Se devuelve el valor de la característica
            return self.python_gatt_callback.getValue(service_uuid, characteristic_uuid) # Se obtiene el valor de la característica convertido a string

        except Exception as e:
            print("Error encontrado")
            print(f"Error: {e}")

            return "NULL"

    def read_json(self, service_uuid: str, characteristic_uuid: str) -> dict: 
        '''Método que convierte el un string en un archivo JSON y llama al método read()
        Entradas: service_uuid str -> UUID del servicio de la característica
                  characteristic_uuid str -> UUID de la característica
                  data str -> mensaje a enviar
        '''
        # Se llama al método write()
        valor = self.read(service_uuid, characteristic_uuid)
        print(f"Raw string: {valor}")
        dict_json = json.loads(valor)

        return dict_json # Devuelve el diccionario JSON leído
    
    def set_notifications(self, service_uuid: str, characteristic_uuid: str, enable: bool) -> bool:
        '''
        Método para habilitar/deshabilitar notificaciones de una característica del gatt conectado.
        Entradas: service_uuid str -> UUID del servicio de la característica a habilitar/deshabilitar notificaciones
                  characteristic_uuid str -> UUID de la característica a habilitar/deshabilitar notificaciones
                  enable bool -> True para habilitar, False para deshabilitar

        Salida: True de operación ejecutada con éxito, False en caso contrario
        '''
        try: 
            if not self.connected: return False # Error si no hay dispositivo conectado
            if not self.connected_gatt: return False # Error si no hay gatt conectado
        
            # Atributo para habilitar/deshabilitar notificaciones
            if enable:
                payload = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            else:
                payload = BluetoothGattDescriptor.DISABLE_NOTIFICATION_VALUE

            # Identifica la característica de interés del servicio de interés con los UUID's
            for i, char in enumerate(self.discovered_characteristics[service_uuid]):
                if char.getUuid().toString() == characteristic_uuid: 
                    characteristic = char
                    index = i
                    break
            
            # Verifica si la característica es notificable
            car_analyzer = Characteristic_Info(characteristic)
            if not car_analyzer.isNotifiable(): 
                print("Característica no notificable")
                return False

            # Se obtiene el descriptor de la característica
            descriptor_UUID: str = ""
            descriptors: BluetoothGattDescriptor = characteristic.getDescriptors() # type: ignore
            for i, descriptor in enumerate(descriptors): # CUÁL DESCRIPTOR AGARRAR? SE TOMA EL ÚLTIMO POR DEFAULT
                print(i)
                descriptor_UUID = descriptor.getUuid().toString() # Se obtiene el UUID del descriptor
            if descriptor_UUID == "": 
                print("No se se encontró un descriptor")
                return False
            CCCD = characteristic.getDescriptor(UUIDClass.fromSrting(descriptor_UUID)) # Descriptor de la característica

            # Se habilita la notificación
            success: bool = self.connected_gatt.setCharacteristicNotification(characteristic, enable) # Se revisa de errores

            if not success: 
                print("Error al habilitar notificaciones")
                return False
            
            # Se escribe el valor del descriptor
            CCCD.setValue(payload)
            self.connected_gatt.writeDescriptor(CCCD) 
            
            sleep(0.1) # Pequeña espera para el callback

            # Operación terminada con éxito
            success = self.python_gatt_callback.notification_flag()
            if success: print("Notificaciones habilitadas")
            else: print("Error al habilitar notificaciones")
            return success

        except Exception as e:
            print(f"Error: {e}")