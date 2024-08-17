// Código para la ESP32 que maneja el protocolo BLE e I2C como maestro. 

// Importación de librerías
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>
#include "Wire.h"

// ------------------------------------------ Variables de uso general ---------------------------------------------
// Pin del LED integrado en la ESP32
#define LED_PIN 2

// ----------------------------------- Declaración de funciones prototipo ------------------------------------------
void sendI2CMessage(uint8_t slaveAddress, char* message);
String readI2CMessage(uint8_t slaveAddress, uint8_t len);

// --------------------------------------- Constantes de uso general -----------------------------------------------
// ------- Constantes para BLE -------
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
#define CHARACTERISTIC_UUID_PV "0000000b-0000-1000-8000-00805f9b34fa"      // Característica para recibir el valor de la variable de proceso
#define CHARACTERISTIC_UUID_ALL_PV "0000000e-0000-1000-8000-00805f9b34fa"  // Característica para leer todas las variables de proceso en modo de monitoreo

// Servicio para el modo de comando 
#define SERVICE_UUID_COMMAND "00000003-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_MODE "0000000c-0000-1000-8000-00805f9b34fa" // Característica para definir qué variable de proceso analizar

// ------- Constantes para I2C -------
#define SLAVE_ADDRESS 0x55  // Dirección del esclavo I2C para Right Leg
#define BUFFER_SIZE 300     // Tamaño del buffer para recibir datos ELIMINAR

// ------------------------------------------ Set up para BLE ------------------------------------------
// ------- Variables para BLE -------
// Variables del servidor
BLEServer* pServer;
BLEAdvertising* pAdvertising;

// Declara variables de las características
BLECharacteristic *pCharacteristic_PI;
BLECharacteristic *pCharacteristic_LEVEL;
BLECharacteristic *pCharacteristic_SP;
BLECharacteristic *pCharacteristic_PV;
BLECharacteristic *pCharacteristic_VAR;   // NO SE USA 
BLECharacteristic *pCharacteristic_MODE;

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

// ------- Clases de callbacks para las características BLE -------
// Clase que maneja los eventos de conexión y desconexión
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { // Cuando se conecta
    digitalWrite(LED_PIN, HIGH); 
    Serial.println("Cliente conectado");
  }

  void onDisconnect(BLEServer* pServer) { // Cuando se desconecta
    digitalWrite(LED_PIN, LOW);  
    Serial.print("Cliente desconectado ... ");
    delay(100); // Delay para reiniciar advertising
    Serial.println("Reiniciando advertising.");
    pAdvertising->start();               // Reinicia el advertisement
  }
};

// Clase callback que maneja los datos escritos en la característica de parámetros PI
class BLECallback_PI: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    // Método que notifica cuando el cliente lee la característica
    Serial.println("Característica PI leída por el cliente");
  } // fin de onRead

  void onWrite(BLECharacteristic *pCharacteristic) {
    // Método que recibe un nuevo valor de la característica
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.println("Característica escrita: " + String(value.c_str()));

    // Procesa los datos recibidos en formato JSON
    StaticJsonDocument<450> jsonrec; // Longitud para procesar el JSON
    DeserializationError error = deserializeJson(jsonrec, value);
    /* Ejemplo de archivo
      {
        "limb": "Right leg",
        "motor1": {
            "pos": {"kc": "100", "ti": "50"},
            "vel": {"kc": "100", "ti": "50"},
            "cur": {"kc": "100", "ti": "50"},
            },
        "motor2": {
            "pos": {"kc": "100", "ti": "50"},
            "vel": {"kc": "100", "ti": "50"},
            "cur": {"kc": "100", "ti": "50"},
            },
        "motor3": {
            "pos": {"kc": "100", "ti": "50"},
            "vel": {"kc": "100", "ti": "50"},
            "cur": {"kc": "100", "ti": "50"},
            },
      }
    
      Después de recibirlo se enviarán en 5 paquetes por I2C
      Paquete 1: Indicador de INICIO con limb
      Paquete 2: Parámetros de motor 1
      Paquete 3: Parámetros de motor 2
      Paquete 4: Parámetros de motor 3
      Paquete 5: Indicador de FINAL
      (Esta lógica se tiene que procesar en la TIVA para esta recepción de mensajes)

      Ejemplo de mensaje por I2C para que la TIVA lo reciba: 
      // SI NO SE PUEDE JSON
      "1, P, 100, 50, V, 100, 50, C, 100, 50" 

      // SI SE PUEDE JSON 
      "motor1": {               
        "pos": {"kc": "100", "ti": "50"},
        "vel": {"kc": "100", "ti": "50"},
        "cur": {"kc": "100", "ti": "50"},
      },
    */
    
    // Se guarda la extremidad recibida
    selected_limb = String(jsonrec["limb"]);
    Serial.println("Mensaje recibido: ");
    serializeJson(jsonrec, Serial);

    // Valida el formato del JSON
    if (error) {
      Serial.print("Error al analizar JSON: ");
      Serial.println(error.c_str());
      return;
    }
    
    // PROCESAMIENTO PARA GUARDAR LOS PARÁMETROS RECIBIDOS

    // Enviar notificación de éxito en formato JSON
    StaticJsonDocument<20> jsonrep;
    jsonrep["response"] = "Success";
    char responseBuffer[20];
    serializeJson(jsonrep, responseBuffer);
    pCharacteristic->setValue("Write response");
    pCharacteristic->notify();
  } // fin de onWrite
};

// Clase callback que maneja los datos escritos en la característica del parámetro SP
class BLECallback_SP: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    // Método que notifica cuando el cliente lee la característica
    Serial.println("Característica SP leída por el cliente");
  } // fin de onRead

  void onWrite(BLECharacteristic *pCharacteristic) {
    // Método que recibe un nuevo valor de la característica
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.println("Característica escrita: " + String(value.c_str()));

    // Procesa los datos recibidos en formato JSON
    StaticJsonDocument<180> jsonrec;
    DeserializationError error = deserializeJson(jsonrec, value);

    Serial.println("Mensaje recibido: ");
    serializeJson(jsonrec, Serial);

    // Valida el formato del JSON
    if (error) {
      Serial.print("Error al analizar JSON: ");
      Serial.println(error.c_str());
      return;
    }
    /* Ejemplo de archivo
      {
          "limb": "Right leg", 
          "monitoring": "pos", 
          "motor1": "0",
          "motor2": "0",
          "motor3": "0",
      }

      Ejemplo de mensaje por I2C para que la TIVA lo reciba: 
      // SI NO SE PUEDE JSON
      "l, RL, M, P, M1, 0, M2, 0, M3, 0" 

      // SI SE PUEDE JSON 
      Enviar tal cual se recibió
    */
    
    // Enviar notificación de éxito en formato JSON
    StaticJsonDocument<20> jsonrep;
    jsonrep["response"] = "Success";
    char responseBuffer[20];
    serializeJson(jsonrep, responseBuffer);
    pCharacteristic->setValue("Write response");
    pCharacteristic->notify();
  } // fin de onWrite
};
// Clase callback que maneja los datos escritos en la característica de parámetros PI
class BLECallback_LEVEL: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    // Método que notifica cuando el cliente lee la característica
    Serial.println("Característica LEVELleída por el cliente");
  } // fin de onRead

  void onWrite(BLECharacteristic *pCharacteristic) {
    // Método que recibe un nuevo valor de la característica
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.println("Característica escrita: " + String(value.c_str()));

    // Procesa los datos recibidos en formato JSON
    StaticJsonDocument<30> jsonrec;
    DeserializationError error = deserializeJson(jsonrec, value);

    Serial.println("Mensaje recibido: ");
    serializeJson(jsonrec, Serial);

    // Valida el formato del JSON
    if (error) {
      Serial.print("Error al analizar JSON: ");
      Serial.println(error.c_str());
      return;
    }
    /* Ejemplo de archivo 
      {"asistance_level": "100"}

      Ejemplo de mensaje por I2C para que la TIVA lo reciba: 
      // SI NO SE PUEDE JSON
      "AL, 100" 

      // SI SE PUEDE JSON 
      Enviar tal cual se recibió (debería ser sencillo de procesar)
    */

    // Recibe el valor y se comprueba que no haya errores. 
    level = String((const char*)jsonrec["asistance_level"]);

    // Verifica que encontró el valor
    if (level == "null") {
      Serial.println("Error al mandar los parámetros.");
      return;
    }

    // Impresión de datos
    Serial.print("Nivel de asistencia: ");
    Serial.println(level);

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
    StaticJsonDocument<50> jsonrec;
    DeserializationError error = deserializeJson(jsonrec, value);

    Serial.println("Mensaje recibido: ");
    serializeJson(jsonrec, Serial);

    // Valida el formato del JSON
    if (error) {
      Serial.print("Error al analizar JSON: ");
      Serial.println(error.c_str());
      return;
    }
    /* Ejemplo de archivo
      {"state": "sit_down_stand_up"}

      Ejemplo de mensaje por I2C para que la TIVA lo reciba: 
      // SI NO SE PUEDE JSON
      "S: SDSU" 

      // SI SE PUEDE JSON 
      Enviar tal cual se recibió (debería ser sencillo de procesar)
    */

    // Recibe el valor y se comprueba que no haya errores. 
    state = String(jsonrec["state"]);

    // ELIMINAR
    if (state == "sit_down_stand_up"){
      char* message2 = "ON ";
      sendI2CMessage(SLAVE_ADDRESS, message2);
    }
    if (state == "walk"){
      char* message2 = "OFF";
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
// ------- Variables para I2C -------

char receivedData[BUFFER_SIZE]; // Buffer para almacenar los datos recibidos
uint8_t dataLength = 0; // Longitud de los datos recibidos

// ------- funciones para I2C -------
// Función para mandar un mensaje a través de I2C
void sendI2CMessage(uint8_t slaveAddress, char* message) {
  int length = strlen(message);  // Calcular el tamaño del mensaje
  message[length] = 'X';         // Añadir un terminador X personalizado 
  byte byteArray[length++];      // Crear un arreglo de bytes del mismo tamaño 

  // Se convierte el string a un arreglo de bytes
  for (int i = 0; i < length; i++) {
    byteArray[i] = (byte)message[i];
  }

  // Se imprime el mensaje a mandar
  Serial.print("Sending message: ");
  Serial.println(message);

  // Se manda el mensaje al esclavo I2C
  Wire.beginTransmission(slaveAddress);
  int bytesWritten = Wire.write(byteArray, length);  
  int errorCode = Wire.endTransmission();

  // Se imprime el error si hay uno
  if (errorCode != 0){
    Serial.print("Código de error: ");
    Serial.println(errorCode);
  }
}

String readI2CMessage(uint8_t slaveAddress, uint8_t len){
  Wire.requestFrom(slaveAddress, len);

  // Leer los datos recibidos y almacenarlos en el buffer
  String mensaje_leido = "";

  // Lee los bytes recibidos
  int i = 0;
  while (Wire.available()) {
    const char rec_data = Wire.read();

    // Verificar si el byte recibido es "X"
    if (rec_data == 'X') {
      Serial.println("Byte 'X' recibido. Deteniendo la lectura.");
      break; // Deja de leer
    }

    mensaje_leido += rec_data;
  }

  Serial.print("Datos recibidos: ");
  Serial.println(mensaje_leido);

  return mensaje_leido;
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
  // Crea la característica BLE para recibir el SP del proceso
  pCharacteristic_SP = pService_PARAMS->createCharacteristic(
                                         CHARACTERISTIC_UUID_SP,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic_SP->setCallbacks(new BLECallback_SP());

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
  values_doc["monitoring"] = "pos";
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
  // Se notifica que se debe leer una característica
  pCharacteristic_PV->notify();
}