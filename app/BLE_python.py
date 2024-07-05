from jnius import autoclass
BluetoothAdapter = autoclass('android.bluetooth.BLuetoothAdapter')

bluetooth_adapter = BluetoothAdapter.getDefaultAdapter()
if bluetooth_adapter is None: 
    print("Bluetooth not supported on this device")
else:
    print("Bluetooth supported")

# from jnius import autoclass, cast
# import json

# # Android BLE Classes
# Context = autoclass('android.content.Context')
# BluetoothAdapter = autoclass('android.bluetooth.BluetoothAdapter')
# BluetoothManager = autoclass('android.bluetooth.BluetoothManager')
# BluetoothDevice = autoclass('android.bluetooth.BluetoothDevice')
# BluetoothGatt = autoclass('android.bluetooth.BluetoothGatt')
# BluetoothGattCallback = autoclass('android.bluetooth.BluetoothGattCallback')
# BluetoothGattCharacteristic = autoclass('android.bluetooth.BluetoothGattCharacteristic')
# BluetoothGattService = autoclass('android.bluetooth.BluetoothGattService')
# UUID = autoclass('java.util.UUID')

# class MyGattCallback(BluetoothGattCallback):
#     def __init__(self, library):
#         self.library = library

#     def onConnectionStateChange(self, gatt, status, new_state):
#         if new_state == BluetoothGatt.STATE_CONNECTED:
#             print('Connected to device')
#             gatt.discoverServices()

#     def onServicesDiscovered(self, gatt, status):
#         if status == BluetoothGatt.GATT_SUCCESS:
#             print('Services discovered')
#             # Here you can list all services and characteristics

#     def onCharacteristicWrite(self, gatt, characteristic, status):
#         if status == BluetoothGatt.GATT_SUCCESS:
#             print('Data written to characteristic')

# class BLELibrary:
#     def __init__(self):
#         self.context = cast('android.app.Activity', autoclass('org.kivy.android.PythonActivity').mActivity)
#         self.bluetooth_manager = self.context.getSystemService(Context.BLUETOOTH_SERVICE)
#         self.bluetooth_adapter = self.bluetooth_manager.getAdapter()
#         self.gatt = None

#     def enable_bluetooth(self):
#         if not self.bluetooth_adapter.isEnabled():
#             self.bluetooth_adapter.enable()

#     def start_scan(self):
#         self.bluetooth_adapter.startLeScan(self.le_scan_callback)

#     def le_scan_callback(self, device, rssi, scan_record):
#         print(f'Device found: {device.getName()}, {device.getAddress()}')
#         # For demonstration purposes, connect to the first device found
#         # self.connect_to_device(device.getAddress())

#     def connect_to_device(self, address):
#         '''Given the address of a BLE device, creates a connection to it.'''
#         device = self.bluetooth_adapter.getRemoteDevice(address) # Gets the remote device with its address
#         self.gatt = device.connectGatt(self.context, False, MyGattCallback(self)) # Creates a connection GATT with that device

#     def create_gatt_server(self):
#         # Se definen los UUIDs del service y la characteristic
#         service_uuid = UUID.fromString('0000180d-0000-1000-8000-00805f9b34fb')  # Replace with your custom UUID
#         characteristic_uuid = UUID.fromString('00002a37-0000-1000-8000-00805f9b34fb')  # Replace with your custom UUID

#         # Se crea el service
#         service = BluetoothGattService(service_uuid, BluetoothGattService.SERVICE_TYPE_PRIMARY)
#         # Se crea la characteristic
#         characteristic = BluetoothGattCharacteristic(characteristic_uuid,
#                                                      BluetoothGattCharacteristic.PROPERTY_WRITE,
#                                                      BluetoothGattCharacteristic.PERMISSION_WRITE)
#         # Se añade la characteristic al servicio
#         service.addCharacteristic(characteristic)

#         # Se crea el GATT server
#         self.gatt_server = self.bluetooth_manager.openGattServer(self.context, MyGattCallback(self))

#         # Se añade el servicio al GATT server
#         self.gatt_server.addService(service)

#         # Se muestra el resultado
#         print('GATT server created')

#     def send_json_data(self, data):
#         if self.gatt:
#             service = self.gatt.getService(UUID.fromString('0000180d-0000-1000-8000-00805f9b34fb'))
#             characteristic = service.getCharacteristic(UUID.fromString('00002a37-0000-1000-8000-00805f9b34fb'))

#             json_data = json.dumps(data)
#             characteristic.setValue(json_data.encode('utf-8'))
#             self.gatt.writeCharacteristic(characteristic)

#     def receive_data(self, characteristic, data):
#         # Parse JSON data received
#         json_data = json.loads(data.decode('utf-8'))
#         print(f'Received data: {json_data}')

#     def gatt_services_to_json(self):
#         services = self.gatt.getServices()
#         gatt_services = []
#         for service in services:
#             characteristics = service.getCharacteristics()
#             char_list = []
#             for characteristic in characteristics:
#                 char_list.append({
#                     'uuid': characteristic.getUuid().toString(),
#                     'properties': characteristic.getProperties(),
#                     'permissions': characteristic.getPermissions()
#                 })
#             gatt_services.append({
#                 'uuid': service.getUuid().toString(),
#                 'type': service.getType(),
#                 'characteristics': char_list
#             })
#         return json.dumps(gatt_services, indent=2)

# # Usage example
# if __name__ == '__main__':
#     ble_library = BLELibrary()
#     ble_library.enable_bluetooth()
#     ble_library.start_scan()

#     # Simulate sending JSON data
#     data_to_send = {
#         'key1': 'value1',
#         'key2': 'value2'
#     }

#     ble_library.send_json_data(data_to_send)

#     # Convert GATT services to JSON
#     gatt_services_json = ble_library.gatt_services_to_json()
#     print(gatt_services_json)
