// Código para la ESP32 que maneja el protocolo BLE e I2C como maestro. 

// Importación de librerías
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>  
#include <ArduinoJson.h>
#include "Wire.h"

// ------------------------------------------ Variables de uso general ---------------------------------------------
// Pin del LED integrado en la ESP32
#define LED_PIN 2

// ----------------------------------- Declaración de funciones prototipo ------------------------------------------
void sendI2CMessage(uint8_t slaveAddress, const char* message);
String readI2CMessage(uint8_t slaveAddress);

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
#define SLAVE_ADDRESS 0x55     // Dirección del esclavo I2C para Right Leg
#define BUFFER_SIZE 300        // Tamaño del buffer para recibir datos ELIMINAR
#define I2C_BUFFER_LENGTH 128  // Tamaño máximo del buffer modificado en la ESP32
// ------------------------------------------ Set up para BLE ------------------------------------------
// ------- Variables para BLE -------
bool client_connected = false;

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
    client_connected = true;
  }

  void onDisconnect(BLEServer* pServer) { // Cuando se desconecta
    digitalWrite(LED_PIN, LOW);  
    client_connected = false;
    Serial.println("Cliente desconectado.");
    delay(100); // Delay para reiniciar advertising
    pAdvertising->start();               // Reinicia el advertisement
  }
};

// Clase callback que maneja los datos escritos en la característica de parámetros PI
class BLECallback_PI: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    // Método que notifica cuando el cliente lee la característica
    Serial.println("Característica PI leída");
  } // fin de onRead

  void onWrite(BLECharacteristic *pCharacteristic) {
    // Método que recibe un nuevo valor de la característica
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.println("Característica PI escrita: " + String(value.c_str()));

    // Procesa los datos recibidos en formato JSON
    StaticJsonDocument<450> jsonrec; // Longitud para procesar el JSON
    DeserializationError error = deserializeJson(jsonrec, value);

    // Revisión de errores 
    // La validación del sub archivo json dentro de motor1, motor2 y motor3 se hace en la Tiva
    if (error) { // Error en la deserialización
      Serial.print("Error al analizar JSON: ");
      Serial.println(error.c_str());
      return;
    }
    else if (!jsonrec.containsKey("limb") || !jsonrec.containsKey("motor1") || !jsonrec.containsKey("motor2") || !jsonrec.containsKey("motor3")){ // Error de key
      Serial.println("Información incorrecta en el archivo JSON");
      return;
    }
    else if (!jsonrec["limb"].is<String>()){ // Error de tipo de dato
      Serial.println("Tipo de dato erróneo en limb");
      return;  
    }

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
    
      Después de recibirlo se enviarán en 3 paquetes por I2C
      Paquete 1: Parámetros de motor 1
      Paquete 2: Parámetros de motor 2
      Paquete 3: Parámetros de motor 3

      Ejemplo de mensaje por I2C para que la TIVA lo reciba: 
      {
        "T": "B"
        "motor1": {               
          "pos": {"kc": "100", "ti": "50"},
          "vel": {"kc": "100", "ti": "50"},
          "cur": {"kc": "100", "ti": "50"},
        },
      }
    */
    // --------------- Procesamiento del archivo json para mandar ---------------
    DynamicJsonDocument jsonsend(160);
    String stringsend = "";

    // JSON del motor 1
    jsonsend["motor1"] = jsonrec["motor1"];
    jsonsend["T"] = "B";
    serializeJson(jsonsend, stringsend);
    stringsend += '\n'; // Se añade el caracter terminador
    sendI2CMessage(SLAVE_ADDRESS, stringsend.c_str());
    delay(300); // Se espera 300 ms para mandar el siguiente

    // JSON del motor 2
    jsonsend.remove("motor1");
    jsonsend["motor2"] = jsonrec["motor2"];
    jsonsend["T"] = "C";
    stringsend = "";
    serializeJson(jsonsend, stringsend);
    stringsend += '\n'; // Se añade el caracter terminador
    sendI2CMessage(SLAVE_ADDRESS, stringsend.c_str());
    delay(300); // Se espera 300 ms para mandar el siguiente

    // JSON del motor 3
    jsonsend.remove("motor2");
    jsonsend["motor3"] = jsonrec["motor3"];
    jsonsend["T"] = "D";
    stringsend = "";
    serializeJson(jsonsend, stringsend);
    stringsend += '\n'; // Se añade el caracter terminador
    sendI2CMessage(SLAVE_ADDRESS, stringsend.c_str());
    delay(300); // Se espera 300 ms para mandar el siguiente
    
    // Enviar notificación de éxito en formato JSON
    pCharacteristic->notify();
  } // fin de onWrite
};

// Clase callback que maneja los datos escritos en la característica del parámetro SP
class BLECallback_SP: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    // Método que notifica cuando el cliente lee la característica
    Serial.println("Característica SP leída");
  } // fin de onRead

  void onWrite(BLECharacteristic *pCharacteristic) {
    // Método que recibe un nuevo valor de la característica
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.println("Característica escrita: " + String(value.c_str()));

    // Procesa los datos recibidos en formato JSON
    StaticJsonDocument<100> jsonrec;
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
      {
        "T": "E",
      {
        "monitoring": "pos", 
        "motor1": "0",
        "motor2": "0",
        "motor3": "0",
      }
    */

    // --------------- Procesamiento del archivo json para mandar ---------------
    // Formato y transmisión para I2C
    jsonrec.remove("limb"); // Se quita la limb del mensaje
    jsonrec["T"] = "E";     // Se añade el tipo de mensaje 
    String stringsend = "";
    serializeJson(jsonrec, stringsend);
    stringsend += '\n'; // Se añade el caracter terminador
    sendI2CMessage(SLAVE_ADDRESS, stringsend.c_str());

    // Enviar notificación de éxito en formato JSON
    pCharacteristic->notify();
  } // fin de onWrite
};
// Clase callback que maneja los datos escritos en la característica de parámetros PI
class BLECallback_LEVEL: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    // Método que notifica cuando el cliente lee la característica
    Serial.println("Característica LEVEL leída");
  } // fin de onRead

  void onWrite(BLECharacteristic *pCharacteristic) {
    // Método que recibe un nuevo valor de la característica
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.println("Característica nueva en BLE: " + String(value.c_str()));

    // Procesa los datos recibidos en formato JSON
    DynamicJsonDocument jsonrec(40);
    DeserializationError error = deserializeJson(jsonrec, value);

    // --- Revisión de errores ---
    if (error) { // Error en la deserialización
      Serial.print("Error al analizar JSON: ");
      Serial.println(error.c_str());
      return;
    }
    else if (!jsonrec.containsKey("assistance_level")){ // Error de key
      Serial.println("Tipo no encontrado en el JSON");
      return;
    }
    else if (!jsonrec["assistance_level"].is<String>()){ // Error de tipo de dato
      Serial.println("Tipo de dato erróneo");
      return;  
    }

    /* Ejemplo de archivo 
      {"asistance_level": "100"}

      Ejemplo de mensaje por I2C para que la TIVA lo reciba: 
      { "T": "A",
        "asistance_level": "100"
      }
    */

    // --------------- Procesamiento del archivo json para mandar ---------------
    // Formato y transmisión para I2C
    jsonrec["T"] = "A"; // Se añade el tipo de mensaje 
    String stringsend = "";
    serializeJson(jsonrec, stringsend);
    stringsend += '\n'; // Se añade el caracter terminador
    sendI2CMessage(SLAVE_ADDRESS, stringsend.c_str());

    // Enviar notificación de éxito en formato JSON
    pCharacteristic_LEVEL->notify();
  } // fin de onWrite
};

// Clase de Callback para manejar la información de la característica de la variable de proceso 
class BLECallback_PV : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    // Método que notifica cuando el cliente lee la característica
    Serial.println("Característica PV leída");
  } // fin de onRead
  void onWrite(BLECharacteristic *pCharacteristic) {
    // Método que recibe un nuevo valor de la característica
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.println("Característica PV escrita: " + String(value.c_str()));
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
      {"state": "walk"}
      {"state": "stop"}

      Ejemplo de mensaje por I2C para que la TIVA lo reciba: 
      {"state": "stop", "T", "H"}
    */

    // Recibe el valor y se comprueba que no haya errores. 
    state = String(jsonrec["state"]);

    // --------------- Procesamiento del archivo json para mandar ---------------
    // Formato y transmisión para I2C
    jsonrec["T"] = "H"; // Se añade el tipo de mensaje 
    jsonrec.remove("limb");
    String stringsend = "";
    serializeJson(jsonrec, stringsend);
    stringsend += '\n'; // Se añade el caracter terminador
    sendI2CMessage(SLAVE_ADDRESS, stringsend.c_str());

    // Enviar notificación de éxito en formato JSON
    pCharacteristic->notify();
  }
};

// ------------------------------------- Set up para I2C -------------------------------------
// ------- Variables para I2C -------

char receivedData[BUFFER_SIZE]; // Buffer para almacenar los datos recibidos
uint8_t dataLength = 0; // Longitud de los datos recibidos

// ------- funciones para I2C -------
// Función para reiniciar el bus I2C ante algún error
void resetI2C() {
  Wire.end();     // Finaliza la comunicación I2C actual
  delay(5);       // Ligero delay
  Wire.begin();   // Reinicia la comunicación I2C
}

// Función para mandar un mensaje a través de I2C
void sendI2CMessage(uint8_t slaveAddress, const char* message) {
  int length = strlen(message);  // Calcular el tamaño del mensaje
  byte byteArray[length];        // Crear un arreglo de bytes del mismo tamaño 
  int buffer_size = 32;          // Número máximo de bytes a transmitir (por protocolo)
  int errorCode;

  do { // Intento de mandar información por I2C
    for (int i = 0; i < length; i += buffer_size) { 
      Wire.beginTransmission(slaveAddress);                   // Iniciar transmisión con la dirección del esclavo
      
      // Cantidad de bytes a mandar
      int bytes_to_send = min(buffer_size, length - i);

      // Envía el fragmento del mensaje
      for (int j = 0; j < bytes_to_send; j++) {
        Wire.write(message[i + j]);
      }
      
      errorCode = Wire.endTransmission();
      // Se imprime el error si hay uno
      if (errorCode == 5){
        Serial.println("retry");
        resetI2C();
      }
      
      delay(10);  // delay para evitar saturación del bus I2C
    }
  } while (errorCode == 5); // Se repite si encontró un erro
} //  fin de la función

String readI2CMessage(uint8_t slaveAddress) {
  String message = "";
  bool messageComplete = false;

  while (!messageComplete) {
    Wire.requestFrom(slaveAddress, 32);  // Solicita hasta 32 bytes
    while (Wire.available()) {
      char receivedChar = Wire.read();
      if (receivedChar == '\n') {
        messageComplete = true;
        break;
      }
      message += receivedChar;
    }
    delay(50);
  }

  return message;
}

void read_PV(){
  /* Función que lee la variable de proceso de la Tiva y la manda por BLE
  1 - Manda un indicador en formato JSON de qué información se quiere leer {"T":"F"} 
  2 - Manda un request de lectura en buffers de 32 bytes por I2C
  3 - Termina la lectura de información cuando se recibe un caracter terminador \n
  4 - Recibe el JSON stirng, lo convierte a JSON file y se asegura de haya llegado correctamente 
  (Formato de ejemplo en la característica PV)
  5 - Serializa el JSON, se asigna el valor a la característica y se notifica al cliente 
  */ 

  // Se indica qué se desea leer
  DynamicJsonDocument jsonsend(15);
  String stringsend = "";
  jsonsend["T"] = "F"; // {"T":"F"}
  serializeJson(jsonsend, stringsend);
  stringsend += '\n'; // Se añade el caracter terminador
  sendI2CMessage(SLAVE_ADDRESS, stringsend.c_str());

  delay(50); // Pequeño delay para que la Tiva procese el JSON

  // Hace el request al esclavo
  String stringread = readI2CMessage(SLAVE_ADDRESS);

  // Se deserializa el JSON
  StaticJsonDocument<100> jsonrec; 
  DeserializationError error = deserializeJson(jsonrec, stringread);

  // Se revisan errores
  if (error) {
    Serial.print("Error al analizar JSON: ");
    Serial.println(error.c_str());
    return;
  }
  else if (!jsonrec.containsKey("monitoring") || !jsonrec.containsKey("motor1") || !jsonrec.containsKey("motor2") || !jsonrec.containsKey("motor3")){
      Serial.print("Información no encontrada");
      return;
  }

  if (client_connected){ // Se mandan los datos a la app si está conectada
    Serial.println("Notificación");
    String stringsend = "";
    serializeJson(jsonrec, stringsend);
    pCharacteristic_PV->setValue(stringsend.c_str()); // Se mandan los valores por BLE

    // Se notifica sobre la característica al cliente
    pCharacteristic_PV->notify();
  }
}

void setup() {
  // Inicializa el puerto serie para la depuración
  Serial.begin(115200);
  Serial.println("Iniciando BLE");

  Wire.begin();  // SDA = GPIO 21, SCL = GPIO 22 by default on ESP32
  Wire.setClock(400000);  // Establece la velocidad del I2C a 100 kHz

  // Inicializa el pin del LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  

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
  pCharacteristic_PV->addDescriptor(new BLE2902()); // Se añade un descriptor

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
  StaticJsonDocument<20> doc;
  doc["init"] = "0";
  char buffer[200];
  serializeJson(doc, buffer);
  pCharacteristic_PI->setValue(buffer);
  pCharacteristic_MODE->setValue(buffer);
  
  // Inicialización de la característica PI
  StaticJsonDocument<200> values_doc;
  char values_buffer[200];
  values_doc["limb"] = "Right leg";
  values_doc["monitoring"] = "pos";
  values_doc["motor1"] = "100";
  values_doc["motor2"] = "100";
  values_doc["motor3"] = "100";
  serializeJson(values_doc, values_buffer);
  pCharacteristic_PV->setValue(values_buffer);

  // Inicia el servicio BLE
  pService_PARAMS->start();
  pService_PV->start();
  pService_MODE->start();

  // Habilita la publicidad del servidor BLE y sus servicios
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID_PARAMS);
  pAdvertising->addServiceUUID(SERVICE_UUID_PROCESS);
  pAdvertising->addServiceUUID(SERVICE_UUID_COMMAND);
  pAdvertising->start();
  
  Serial.println("Servidor BLE iniciado.");
}

void loop() {
  // delay(100); // Lectuda de PV cada 100 ms
  // read_PV();

  // Intento de notificaciones cada 500 ms
  delay(500);
  if (client_connected){
    Serial.println("Notificando");
    pCharacteristic_PV->notify();
  }
}