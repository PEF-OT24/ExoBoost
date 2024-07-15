#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // UUID del servicio
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8" // UUID de la característica

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

const int LED_PIN = 2; // LED integrado en la ESP32

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      digitalWrite(LED_PIN, HIGH); // Encender el LED
      Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      digitalWrite(LED_PIN, LOW); // Apagar el LED
      Serial.println("Device disconnected");
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      std::string value = std::string(pCharacteristic->getValue().c_str());

      if (value.length() > 0) {
        Serial.println("Received Value: " + String(value.c_str()));

        // Deserializar JSON
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, value.c_str());

        if (error) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.f_str());
          return;
        }

        // Acceder a los valores del JSON
        const char* key = doc["key"];
        int number = doc["number"];

        // Imprimir los valores recibidos
        Serial.println("key: " + String(key));
        Serial.println("number: " + String(number));
      }
    }
};

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT); // Configurar el pin del LED como salida
  digitalWrite(LED_PIN, LOW); // Asegurarse de que el LED esté apagado al inicio

  BLEDevice::init("ESP32_BLE_Server");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("Hello World");
  pService->start();

  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");
}

void loop() {
  if (deviceConnected) {
    pCharacteristic->notify(); // Notificar a los clientes conectados
  }
  else{Serial.println("Not connected yet.");}
  delay(1000); // Retardo de 1 segundo para evitar mensajes de depuración excesivos
}
