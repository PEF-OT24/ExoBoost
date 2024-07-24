#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>

// Definición del nombre del dispositivo y UUIDs para el servicio y la característica
#define DEVICE_NAME "ESP32_BLE_Server"
#define SERVICE_UUID "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-210987654321"

// Pin del LED integrado en la ESP32
#define LED_PIN 2

BLEServer* pServer;
BLEAdvertising* pAdvertising;

// Clase que maneja los eventos de conexión y desconexión
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    digitalWrite(LED_PIN, HIGH);  // Enciende el LED
    Serial.println("Cliente conectado");
  }

  void onDisconnect(BLEServer* pServer) {
    digitalWrite(LED_PIN, LOW);  // Apaga el LED
    Serial.println("Cliente desconectado");
    delay(0.5);
    Serial.println("Reiniciando advertising...");
    pAdvertising->start();  // Reinicia la publicidad
  }
};

// Clase que maneja los eventos de lectura y escritura en la característica de parámetros de sintonización
class BLECallbacks: public BLECharacteristicCallbacks {

  void onRead(BLECharacteristic *pCharacteristic) {
    // Método que notifica cuando el cliente lee la característica
    Serial.println("Característica leída por el cliente");
  }

  void onWrite(BLECharacteristic *pCharacteristic) {
    // Método que recibe un nuevo valor de la característica
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.println("Característica escrita: " + String(value.c_str()));

    // Procesa los datos recibidos en formato JSON
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, value);
    

    // Valida el formato del JSON
    if (error) {
      Serial.print("Error al analizar JSON: ");
      Serial.println(error.c_str());
      return;
    }
    else{
      Serial.println("Mensaje JSON recibido: ");
      serializeJson(doc, Serial);
    }

    // Enviar notificación de éxito en formato JSON
    // StaticJsonDocument<200> responseDoc;
    // responseDoc["response"] = "Success";
    // char responseBuffer[200];
    // serializeJson(responseDoc, responseBuffer);
    pCharacteristic->setValue("Write response");
    pCharacteristic->notify();
  }
};

void setup() {
  // Inicializa el puerto serie para la depuración
  Serial.begin(115200);
  Serial.println("Iniciando el servidor BLE...");

  // Inicializa el pin del LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Asegúrate de que el LED esté apagado al inicio

  // Inicializa el dispositivo BLE y el servidor
  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Crea el servicio BLE 
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Crea la característica BLE para recibir datos del PI
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic->setCallbacks(new BLECallbacks());

  // Añade un valor inicial a la característica en formato JSON
  StaticJsonDocument<200> doc;
  doc["mensaje"] = "Hola, cliente!";
  char buffer[200];
  serializeJson(doc, buffer);
  pCharacteristic->setValue(buffer);

  // Inicia el servicio BLE
  pService->start();

  // Habilita la publicidad del servidor BLE
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  
  Serial.println("Servidor BLE iniciado y esperando conexiones...");
}

void loop() {
  // El loop está vacío ya que los eventos son manejados por las clases de callbacks
  delay(0.001);
}
