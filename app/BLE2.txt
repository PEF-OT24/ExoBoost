from jnius import autoclass, cast, PythonJavaClass, java_method
import json

# Importar clases necesarias de Android
Context = autoclass('android.content.Context')
BluetoothAdapter = autoclass('android.bluetooth.BluetoothAdapter')
BluetoothManager = autoclass('android.bluetooth.BluetoothManager')
BluetoothDevice = autoclass('android.bluetooth.BluetoothDevice')
BluetoothGatt = autoclass('android.bluetooth.BluetoothGatt')
BluetoothGattCallback = autoclass('android.bluetooth.BluetoothGattCallback')
BluetoothGattCharacteristic = autoclass('android.bluetooth.BluetoothGattCharacteristic')
BluetoothGattService = autoclass('android.bluetooth.BluetoothGattService')
UUID = autoclass('java.util.UUID')

class MyGattCallback(PythonJavaClass):
    __javainterfaces__ = ['android/bluetooth/BluetoothGattCallback']

    def __init__(self, library):
        super().__init__()
        self.library = library

    @java_method('(Landroid/bluetooth/BluetoothGatt;II)V')
    def onConnectionStateChange(self, gatt, status, new_state):
        if new_state == BluetoothGatt.STATE_CONNECTED:
            print('Connected to device')
            gatt.discoverServices()
        elif new_state == BluetoothGatt.STATE_DISCONNECTED:
            print('Disconnected from device')

    @java_method('(Landroid/bluetooth/BluetoothGatt;I)V')
    def onServicesDiscovered(self, gatt, status):
        if status == BluetoothGatt.GATT_SUCCESS:
            print('Services discovered')
            # Aquí puedes listar todos los servicios y características

    @java_method('(Landroid/bluetooth/BluetoothGatt;Landroid/bluetooth/BluetoothGattCharacteristic;I)V')
    def onCharacteristicWrite(self, gatt, characteristic, status):
        if status == BluetoothGatt.GATT_SUCCESS:
            print('Data written to characteristic')

class BLELibrary:
    def __init__(self):
        # Obtener el contexto de la actividad actual
        self.context = cast('android.app.Activity', autoclass('org.kivy.android.PythonActivity').mActivity)
        # Obtener el servicio BluetoothManager
        self.bluetooth_manager = self.context.getSystemService(Context.BLUETOOTH_SERVICE)
        # Obtener el adaptador Bluetooth
        self.bluetooth_adapter = self.bluetooth_manager.getAdapter()
        self.gatt = None
        self.devices_found = []

    def enable_bluetooth(self):
        """Habilitar Bluetooth si no está habilitado."""
        if not self.bluetooth_adapter.isEnabled():
            self.bluetooth_adapter.enable()

    def start_scan(self):
        """Iniciar el escaneo de dispositivos BLE."""
        self.bluetooth_adapter.startLeScan(self.le_scan_callback)
        print("Scanning started")

    def stop_scan(self):
        """Detener el escaneo de dispositivos BLE."""
        self.bluetooth_adapter.stopLeScan(self.le_scan_callback)
        print("Scanning stopped")

    def le_scan_callback(self, device, rssi, scan_record):
        """Callback que se llama cuando se encuentra un dispositivo durante el escaneo."""
        device_info = {'name': device.getName(), 'address': device.getAddress()}
        print(f'Device found: {device_info}')
        self.devices_found.append(device_info)

    def connect_to_device(self, address):
        """Conectarse a un dispositivo BLE dado su dirección MAC."""
        device = self.bluetooth_adapter.getRemoteDevice(address)
        self.gatt = device.connectGatt(self.context, False, MyGattCallback(self))

    def create_gatt_service(self, service_uuid, characteristic_uuid):
        """Crear un servicio GATT con una característica."""
        service_uuid = UUID.fromString(service_uuid)
        characteristic_uuid = UUID.fromString(characteristic_uuid)

        service = BluetoothGattService(service_uuid, BluetoothGattService.SERVICE_TYPE_PRIMARY)
        characteristic = BluetoothGattCharacteristic(characteristic_uuid,
                                                     BluetoothGattCharacteristic.PROPERTY_WRITE,
                                                     BluetoothGattCharacteristic.PERMISSION_WRITE)
        service.addCharacteristic(characteristic)

        # Agregar el servicio al servidor GATT
        self.gatt_server = self.bluetooth_manager.openGattServer(self.context, MyGattCallback(self))
        self.gatt_server.addService(service)
        print('GATT server created')

    def send_json_data(self, data):
        """Enviar datos en formato JSON a una característica BLE."""
        if self.gatt:
            service = self.gatt.getService(UUID.fromString('0000180d-0000-1000-8000-00805f9b34fb'))
            characteristic = service.getCharacteristic(UUID.fromString('00002a37-0000-1000-8000-00805f9b34fb'))

            json_data = json.dumps(data)
            characteristic.setValue(json_data.encode('utf-8'))
            self.gatt.writeCharacteristic(characteristic)

    def receive_data(self, characteristic, data):
        """Recibir datos de una característica BLE."""
        json_data = json.loads(data.decode('utf-8'))
        print(f'Received data: {json_data}')

    def gatt_services_to_json(self):
        """Convertir los servicios GATT a formato JSON."""
        services = self.gatt.getServices()
        gatt_services = []
        for service in services:
            characteristics = service.getCharacteristics()
            char_list = []
            for characteristic in characteristics:
                char_list.append({
                    'uuid': characteristic.getUuid().toString(),
                    'properties': characteristic.getProperties(),
                    'permissions': characteristic.getPermissions()
                })
            gatt_services.append({
                'uuid': service.getUuid().toString(),
                'type': service.getType(),
                'characteristics': char_list
            })
        return json.dumps(gatt_services, indent=2)

    def get_devices_found(self):
        """Obtener la lista de dispositivos encontrados durante el escaneo."""
        return self.devices_found
