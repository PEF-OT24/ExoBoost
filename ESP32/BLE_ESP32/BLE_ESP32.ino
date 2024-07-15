// Librerías para BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// UUIDs para el servicio y la característica BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "00002A3D-0000-1000-8000-00805f9b34fb"

// Pin del LED incorporado
#define LED_BUILTIN 2

// Variables para la conexión Bluetooth
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Clase para el control de eventos de conexión y desconexión BLE
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      digitalWrite(LED_BUILTIN, HIGH); // Enciende el LED cuando se conecta
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      digitalWrite(LED_BUILTIN, LOW); // Apaga el LED cuando se desconecta
    }
};

void setup() {
  // Configuración del pin del LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Asegúrate de que el LED está apagado al inicio

  // Inicialización del dispositivo BLE
  BLEDevice::init("ESP32");

  // Se inicializa y crea el servidor BLE
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Se crea el servicio BLE
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Se crea la característica BLE
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Creación de un Descriptor BLE
  pCharacteristic->addDescriptor(new BLE2902());

  // Se inicia el servicio
  pService->start();

  // Se inicia el proceso de publicidad
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
}

void loop() {
  // Proceso de desconexión
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // Espere un momento antes de reiniciar la publicidad
      pServer->startAdvertising(); // Reiniciar advertising
      oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }
}
