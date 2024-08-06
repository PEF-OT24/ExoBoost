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

// Declara variables del servidor
BLEServer* pServer;
BLEAdvertising* pAdvertising;

String selected_limb; // Articulación seleccionada actual 
String state; // Estado del proceso actual
// Declara variables para los parámetros
String motor1_kc;
String motor1_ti;
String motor1_sp;
String motor2_kc;
String motor2_ti;
String motor2_sp;
String motor3_kc;
String motor3_ti;
String motor3_sp;

// Declara variables para la variable de proceso
String motor1_pv;
String motor2_pv;
String motor3_pv;

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

// Clase callback que maneja los datos escritos en la característica de parámetros PI
class BLECallback_PI: public BLECharacteristicCallbacks {
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
    StaticJsonDocument<200> motor1_params = jsonrec["motor1"];
    StaticJsonDocument<200> motor2_params = jsonrec["motor2"];
    StaticJsonDocument<200> motor3_params = jsonrec["motor3"];
    selected_limb = String((const char*)jsonrec["limb"]);

    if (motor1_params == "null" or motor2_params == "null" or motor3_params == "null" or selected_limb == "null") {
      Serial.println("Error al mandar los parámetros.");
      return;
    }

    motor1_kc = String(motor1_params["kc"]);
    motor1_ti = String(motor1_params["ti"]);
    motor1_sp = String(motor1_params["sp"]);
  
    motor2_kc = String(motor2_params["kc"]);
    motor2_ti = String(motor2_params["ti"]);
    motor2_sp = String(motor1_params["sp"]);
    
    motor3_kc = String(motor3_params["kc"]);
    motor3_ti = String(motor3_params["ti"]);
    motor3_sp = String(motor1_params["sp"]);

    // Impresión de datos
    Serial.println("\nExtremidad seleccionada: " + selected_limb);
    Serial.println("------------------------------");
    Serial.println("motor 1 - kc: " + motor1_kc);
    Serial.println("motor 1 - ti: " + motor1_ti);
    Serial.println("motor 1 - sp: " + motor1_sp);

    Serial.println("------------------------------");
    Serial.println("motor 2 - kc: " + motor2_kc);
    Serial.println("motor 2 - ti: " + motor2_ti);
    Serial.println("motor 2 - sp: " + motor2_sp);

    Serial.println("------------------------------");
    Serial.println("motor 3 - kc: " + motor3_kc);
    Serial.println("motor 3 - ti: " + motor3_ti);
    Serial.println("motor 3 - sp: " + motor3_sp);

    // Enviar notificación de éxito en formato JSON
    StaticJsonDocument<200> jsonrep;
    jsonrep["response"] = "Success";
    char responseBuffer[200];
    serializeJson(jsonrep, responseBuffer);
    pCharacteristic->setValue("Write response");
    pCharacteristic->notify();
  } // fin de onWrite
};

// Clase de Callback para manejar la información de la característica de la variable de proceso 
class BLECallback_PV : public BLECharacteristicCallbacks {
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
    motor1_pv = String(jsonrec["motor1"]);
    motor2_pv = String(jsonrec["motor2"]);
    motor3_pv = String(jsonrec["motor3"]);
    selected_limb = String(jsonrec["limb"]);

    // Se comprueba que no haya errores
    if (motor1_pv == "null" or motor2_pv == "null" or motor3_pv == "null" or selected_limb == "null") {
      Serial.println("Error al mandar los parámetros.");
      return;
    }

    // Impresión de datos
    Serial.println("\nExtremidad seleccionada: " + selected_limb);
    Serial.println("------------------------------");
    Serial.println("motor 1 - pv: " + motor1_pv);
    Serial.println("motor 2 - pv: " + motor2_pv);
    Serial.println("motor 3 - pv: " + motor3_pv);

    // Enviar notificación de éxito en formato JSON
    StaticJsonDocument<200> jsonrep;
    jsonrep["response"] = "Success";
    char responseBuffer[200];
    serializeJson(jsonrep, responseBuffer);
    pCharacteristic->setValue("Write response");
    pCharacteristic->notify();
  }
};

// Clase de Callback para manejar la información de la característica de la variable del estado 
class BLECallback_MODE : public BLECharacteristicCallbacks {
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
    state = String(jsonrec["state"]);
    selected_limb = String(jsonrec["limb"]);

    // Se comprueba que no haya errores
    if (state == "null" or selected_limb == "null") {
      Serial.println("Error al mandar los parámetros.");
      return;
    }

    // Impresión de datos
    Serial.println("------------------------------");
    Serial.println("\nExtremidad seleccionada: " + selected_limb);
    Serial.println("State: " + state);

    // Enviar notificación de éxito en formato JSON
    StaticJsonDocument<200> jsonrep;
    jsonrep["response"] = "Success";
    char responseBuffer[200];
    serializeJson(jsonrep, responseBuffer);
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

  // Crea el servicio de envío de parámetros de PI
  BLEService *pService_PI = pServer->createService(SERVICE_UUID_PARAMS);
  // Crea la característica BLE para recibir datos del PI
  BLECharacteristic *pCharacteristic_PI = pService_PI->createCharacteristic(
                                         CHARACTERISTIC_UUID_PI,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic_PI->setCallbacks(new BLECallback_PI());

  // Crea el servicio de envío de parámetros de PV
  BLEService *pService_PV = pServer->createService(SERVICE_UUID_PROCESS);
  // Crea la característica BLE para recibir datos de la PV
  BLECharacteristic *pCharacteristic_PV = pService_PV->createCharacteristic(
                                         CHARACTERISTIC_UUID_PV,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic_PV->setCallbacks(new BLECallback_PV());

  // Crea el servicio de envío de valor del estado
  BLEService *pService_MODE = pServer->createService(SERVICE_UUID_COMMAND);
  // Crea la característica BLE para recibir datos de la PV
  BLECharacteristic *pCharacteristic_MODE = pService_MODE->createCharacteristic(
                                         CHARACTERISTIC_UUID_MODE,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic_MODE->setCallbacks(new BLECallback_MODE());

  // Inicialización de las características
  StaticJsonDocument<200> doc;
  doc["mensaje"] = "Hola, cliente!";
  char buffer[200];
  serializeJson(doc, buffer);
  pCharacteristic_PI->setValue(buffer);
  pCharacteristic_PV->setValue(buffer);
  pCharacteristic_MODE->setValue(buffer);

  // Inicia el servicio BLE
  pService_PI->start();
  pService_PV->start();
  pService_MODE->start();

  // Habilita la publicidad del servidor BLE
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID_PARAMS);
  pAdvertising->addServiceUUID(SERVICE_UUID_PROCESS);
  pAdvertising->addServiceUUID(SERVICE_UUID_COMMAND);
  pAdvertising->start();
  
  Serial.println("Servidor BLE iniciado y esperando conexiones...");
}

void loop() {
  // El loop está vacío ya que los eventos son manejados por las clases de callbacks
  delay(0.001);
}