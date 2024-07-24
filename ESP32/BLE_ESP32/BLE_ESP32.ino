#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>

// Definición del nombre del dispositivo
#define DEVICE_NAME "ESP32_BLE_Server"

// Definición de servicios y sus características
#define SERVICE_UUID_PARAMS "00000001-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_PI "0000000a-0000-1000-8000-00805f9b34fa"

// Estos no se usan 
#define SERVICE_UUID_PROCESS "00000002-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_PV "0000000b-0000-1000-8000-00805f9b34fa"

#define SERVICE_UUID_COMMAND "00000003-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_MODE "0000000c-0000-1000-8000-00805f9b34fa"

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
  } // fin de onRead

  void onWrite(BLECharacteristic *pCharacteristic) {
    // Método que recibe un nuevo valor de la característica
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.println("Característica escrita: " + String(value.c_str()));

    // Procesa los datos recibidos en formato JSON
    StaticJsonDocument<200> jsonrec;
    DeserializationError error = deserializeJson(jsonrec, value);

    Serial.println("Mensaje recibido: ");
    serializeJson(jsonrec, Serial);

    // Valida el formato del JSON
    if (error) {
      Serial.print("Error al analizar JSON: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Recibe el valor y se comprueba que no haya errores. 
    String testvalue = jsonrec["motor1"];
    if (testvalue != "null") {
      Serial.println(testvalue);
    }
    else{
      Serial.println("Información no recibida");
    }

    // Enviar notificación de éxito en formato JSON
    StaticJsonDocument<200> jsonrep;
    jsonrep["response"] = "Success";
    char responseBuffer[200];
    serializeJson(jsonrep, responseBuffer);
    pCharacteristic->setValue("Write response");
    pCharacteristic->notify();
  } // fin de onWrite
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
  BLEService *pService = pServer->createService(SERVICE_UUID_PARAMS);

  // Crea la característica BLE para recibir datos del PI
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_PI,
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
  pAdvertising->addServiceUUID(SERVICE_UUID_PARAMS);
  pAdvertising->start();
  
  Serial.println("Servidor BLE iniciado y esperando conexiones...");
}

void loop() {
  // El loop está vacío ya que los eventos son manejados por las clases de callbacks
  delay(0.001);
}