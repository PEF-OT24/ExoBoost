// Código para la ESP32 que maneja el protocolo BLE e I2C como maestro. 

// Importación de librerías
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>
#include "Wire.h"

// -------------------------------- Variables de uso general --------------------------------
// Pin del LED integrado en la ESP32
#define LED_PIN 2

// -------------------------------- Declaración de funciones prototipo ---------------------------
void sendI2CMessage(uint8_t slaveAddress, const char* message);

// ------------------------------------- Set up para BLE -------------------------------------
// ------- Variables para BLE -------
// Nombre del dispositivo
#define DEVICE_NAME "ESP32"

// Definición de servicios y sus características
// Servicio para los parámetros
#define SERVICE_UUID_PARAMS "00000001-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_PI "0000000a-0000-1000-8000-00805f9b34fa"    // Característica de parámetros de PI
#define CHARACTERISTIC_UUID_SP "0000000f-0000-1000-8000-00805f9b34fa"    // Característica de parámetros de SP
#define CHARACTERISTIC_UUID_LEVEL "0000000d-0000-1000-8000-00805f9b34fa" // Característica para el nivel de asistencia del motor

// Servicio para las variables de proceso
#define SERVICE_UUID_PROCESS "00000002-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_PV "0000000b-0000-1000-8000-00805f9b34fa"   // Característica para recibir el valor de la variable de proceso
#define CHARACTERISTIC_UUID_VAR "0000000e-0000-1000-8000-00805f9b34fa"  // Característica para definir qué variable de proceso analizar

// Servicio para el modo de comando 
#define SERVICE_UUID_COMMAND "00000003-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_MODE "0000000c-0000-1000-8000-00805f9b34fa" // Característica para definir qué variable de proceso analizar

// Variables del servidor
BLEServer* pServer;
BLEAdvertising* pAdvertising;

// Declara variables de las características
BLECharacteristic *pCharacteristic_PI;
BLECharacteristic *pCharacteristic_LEVEL
BLECharacteristic *pCharacteristic_PV;
BLECharacteristic *pCharacteristic_VAR;// PARA IMPLEMENTAR
BLECharacteristic *pCharacteristic_MODE

// Variables para guardar los valores recibidos
// Parámetros PI de los motores
String motor1_kc;
String motor1_ti;
String motor1_sp;
String motor2_kc;
String motor2_ti;
String motor2_sp;
String motor3_kc;
String motor3_ti;
String motor3_sp;
String level; // Nivel de asistencia del slider 

// Variable de proceso de los motores
String motor1_pv;
String motor2_pv;
String motor3_pv;
String actual_pv; // PARA IMPLEMENTAR

String state; // Estado del proceso actual

String selected_limb; // Articulación seleccionada actual 

// ------- Clases para BLE -------
// Clase que maneja los eventos de conexión y desconexión
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { // Cuando se conecta
    digitalWrite(LED_PIN, HIGH); 
    Serial.println("Cliente conectado");
  }

  void onDisconnect(BLEServer* pServer) { // Cuando se desconecta
    digitalWrite(LED_PIN, LOW);  
    Serial.print("Cliente desconectado ... ");
    delay(0.5);
    Serial.println("Reiniciando advertising.");
    pAdvertising->start();                // Reinicia la publicidad
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
    motor2_sp = String(motor2_params["sp"]);
    
    motor3_kc = String(motor3_params["kc"]);
    motor3_ti = String(motor3_params["ti"]);
    motor3_sp = String(motor3_params["sp"]);

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

// Clase callback que maneja los datos escritos en la característica de parámetros PI
class BLECallback_LEVEL: public BLECharacteristicCallbacks {
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
    level = String((const char*)jsonrec["asistance_level"]);

    // Verifica que encontró el valor
    if (level == "null") {
      Serial.println("Error al mandar los parámetros.");
      return;
    }

    // Impresión de datos
    Serial.println("\nNivel de asistencia: " + level);
    Serial.println("------------------------------");

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
    Serial.println("\nExtremidad seleccionada: " + selected_limb);
    Serial.println("State: " + state);
    Serial.println("------------------------------");

    // ELIMINAR
    if (state == "sit_down_stand_up"){
      const char* message2 = "ON ";
      sendI2CMessage(SLAVE_ADDRESS, message2);
    }
    if (state == "walk"){
      const char* message2 = "OFF";
      sendI2CMessage(SLAVE_ADDRESS, message2);
    }
    

    // Enviar notificación de éxito en formato JSON
    StaticJsonDocument<200> jsonrep;
    jsonrep["response"] = "Success";
    char responseBuffer[200];
    serializeJson(jsonrep, responseBuffer);
    pCharacteristic->setValue("Write response");
    pCharacteristic->notify();
  }
};

// ------------------------------------- Set up para I2C -------------------------------------
#define SLAVE_ADDRESS 0x55 // Dirección del esclavo I2C (ajustar según sea necesario)
#define BUFFER_SIZE 6     // Tamaño del buffer para recibir datos
char receivedData[BUFFER_SIZE]; // Buffer para almacenar los datos recibidos
uint8_t dataLength = 0; // Longitud de los datos recibidos

int temporal = 100;

// Función para mandar un mensaje a través de I2C
void sendI2CMessage(uint8_t slaveAddress, const char* message) {
  int length = strlen(message);  // Calculate the length of the message
  byte byteArray[length];        // Create a byte array of the same length

  // Variables de estatus
  int bytesWritten; // Longitud de los bytes escritos
  int errorCode;    // Código de error después de escribir

  // Se convierte el string a un arreglo de bytes
  for (int i = 0; i < length; i++) {
    byteArray[i] = (byte)message[i];
  }

  // Se imprime el mensaje a mandar
  Serial.print("Sending message: ");
  Serial.println(message);

  // Se manda el mensaje al esclavo I2C
  Wire.beginTransmission(slaveAddress);
  bytesWritten = Wire.write(byteArray, length);  
  errorCode = Wire.endTransmission();

  // Se formatea la salida para debug
  // Serial.print("Mandando mensaje a dirección: ");
  // Serial.println(slaveAddress);
  Serial.print("Código de error: ");
  Serial.println(errorCode);
  // Serial.print("Bytes escritos: ");
  // Serial.println(bytesWritten);
  // Serial.println("----------------");
}

void readI2CMessage(uint8_t slaveAddress, uint8_t len){
  Wire.requestFrom(slaveAddress, len);

  // Leer los datos recibidos y almacenarlos en el buffer
  // dataLength = 0; // Reiniciar la longitud de los datos
  char mensaje_leido[len+1]; // Arreglo de len bytes para guardar el mensaje

  // Lee los bytes recibidos
  int i = 0;
  while (Wire.available()) {
    if (i < sizeof(mensaje_leido)-1) {
      mensaje_leido[i++] = Wire.read();
    }
  }
  mensaje_leido[i] = '\0';
  // Imprime los datos leídos
  Serial.print("Datos recibidos: ");
  Serial.println(mensaje_leido);
  Serial.println();

  if (strcmp(mensaje_leido, "ON ") == 0) {
    Serial.println("Encender motor");// Mensaje temporal para encnder el motor
    
  } 
  else if (strcmp(mensaje_leido, "OFF") == 0) {
    Serial.println("Apagar motor"); // Mensaje temporal para apagar el motor

  } 
  else {
    Serial.println("Mensaje no reconocido");
  }
  // Mostrar los datos almacenados en el monitor serial
  // Serial.print("Datos recibidos: ");
  // for (uint8_t i = 0; i < dataLength; i++) {
  //   Serial.print(receivedData[i]); // Imprimir cada byte en formato hexadecimal
  //   Serial.print(" ");
  // }
  Serial.println(); // Nueva línea después de los datos
}

void setup() {
  // Inicializa el puerto serie para la depuración
  Serial.begin(115200);
  Serial.println("Iniciando el servidor BLE...");

  Wire.begin();  // SDA = GPIO 21, SCL = GPIO 22 by default on ESP32

  // Inicializa el pin del LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Asegúrate de que el LED esté apagado al inicio

  // Inicializa el dispositivo BLE y el servidor
  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Crea el servicio de envío de parámetros de PI
  BLEService *pService_PARAMS = pServer->createService(SERVICE_UUID_PARAMS);
  // Crea la característica BLE para recibir datos del PI
  pCharacteristic_PI = pService_PARAMS->createCharacteristic(
                                         CHARACTERISTIC_UUID_PI,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic_PI->setCallbacks(new BLECallback_PI());
  // Crea la característica BLE para recibir el nivel de asistencia del slider
  pCharacteristic_LEVEL = pService_PARAMS->createCharacteristic(
                                         CHARACTERISTIC_UUID_LEVEL,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic_LEVEL->setCallbacks(new BLECallback_LEVEL());

  // Crea el servicio de envío de parámetros de PV
  BLEService *pService_PV = pServer->createService(SERVICE_UUID_PROCESS);
  // Crea la característica BLE para recibir datos de la PV
  pCharacteristic_PV = pService_PV->createCharacteristic(
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
  pCharacteristic_MODE = pService_MODE->createCharacteristic(
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
  pCharacteristic_MODE->setValue(buffer);
  
  StaticJsonDocument<200> values_doc;
  char values_buffer[200];
  values_doc["limb"] = "Right leg";
  values_doc["motor1"] = "100";
  values_doc["motor2"] = "100";
  values_doc["motor3"] = "100";
  serializeJson(values_doc, values_buffer);
  pCharacteristic_PV->setValue(values_buffer);

  // Se muestra el valor inicial
  Serial.println("----------------");
  Serial.print("Valor inicial: ");
  Serial.println(values_buffer);
  Serial.println("----------------");

  // Inicia el servicio BLE
  pService_PARAMS->start();
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
  delay(500);
  temporal ++;

  // AUMENTAR EL VALOR PARA VER SI SE LEE CORRECTAMENTE
  StaticJsonDocument<200> values_doc;
  char values_buffer[200];
  values_doc["limb"] = "Right leg";
  values_doc["motor1"] = String(temporal);
  values_doc["motor2"] = String(temporal);
  values_doc["motor3"] = String(temporal);
  serializeJson(values_doc, values_buffer);
  pCharacteristic_PV->setValue(values_buffer);

  // Serial.println(pCharacteristic_PV->getValue());

  // Se manda información por I2C
  // const char* message2 = "Hola, Esclavo!";
  // sendI2CMessage(SLAVE_ADDRESS, message2); // Mandar un mensaje a la TivaC
  // delay(500);
  // readI2CMessage(SLAVE_ADDRESS, 3); // Leer un mensaje
}