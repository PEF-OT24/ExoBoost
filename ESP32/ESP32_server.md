# Código en la ESP32
#### Este código explica el proceso para configurar la ESP32 como servidor BLE, el manejo de comunicación I2C como master y el rol como intermediario entre la aplicación móvil y la TivaC.

#### Para más información, favor de revisar la [documentación de la ESP32](https://rtd-debug-zh.readthedocs.io/en/latest/ambd_arduino/bw16/api_documents/Class%20BLEDevice.html).

## Configuración inicial
#### Es necesario instalar Arduino IDE e instalar el driver para soportar ESP32. En el código se utilizan las siguientes librerias:

``` C
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>  
#include <ArduinoJson.h>
#include "Wire.h"
```

## Configuración de ESP32 como server BLE

#### La ESP32 genera un servidor de BLE que contiene un determinado número de servicios con características que comunican información con la aplicación móvil. Para configurar un servidor se deben seguir los siguientes pasos: 

### 1. Inicialización del servidor

``` C
BLEDevice::init(DEVICE_NAME);
MyServer = BLEDevice::createServer();
MyServer->setCallbacks(new ServerCallbacks());
```

#### En el campo <DEVICE_NAME> se le asigna el nombre al dispositivo para ser descubierto; este atributo forma parte del descriptor GATT.

### 2. Asignar una clase de callbacks para el servidor

#### BLE en la ESP32 funciona a base de métodos callbacks, es decir, interrupciones cuando algún evento de interés sucede. Para configurar las interrupciones del servidor, se crea la siguiente clase personalizable:

``` C
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* MyServer) { 
    // Método que se ejecuta en conexión
    Serial.println("Cliente CONECTADO");
    client_connected = true;
  }

  void onDisconnect(BLEServer* MyServer) {
    // Método que se ejecuta en conexión
    client_connected = false;
    Serial.println("Cliente desconectado.");
  }
};
```

### 3. Crear el servicio

#### Se crea una isntancia de la clase BLEService para cada servicio.
#### Toma como argumento su UUID <SERVICE_UUID>

``` C
BLEService *MyService = MyServer->createService(SERVICE_UUID);
```

### 4. Crear las características
#### Para cada servicio, se crean sus características como instancias de BLECharacteristic. Esta función toma como arguimento el UUID <CHARACTERISTIC_UUID> y los permisos de operaciones de la característica. 

``` C
BLECharacteristic *MyCharacteristic = MyService->createCharacteristic(
                                        CHARACTERISTIC_UUID,
                                        BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_WRITE |
                                        BLECharacteristic::PROPERTY_NOTIFY |
                                        BLECharacteristic::PROPERTY_INDICATE
                                    );
MyCharacteristic->setCallbacks(new BLECharacteristicCallback());
```

### 5. Clase de callback para características

#### Se crea una clase personalizada para las características para manejar las interrupciones en lectura y escritura

``` C
class BLECharacteristicCallback : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    // Método que notifica cuando el cliente lee la característica
    Serial.println("Característica PV leída");
  } 

  void onWrite(BLECharacteristic *pCharacteristic) {
    // Método que recibe un nuevo valor de la característica
    std::string value = std::string(pCharacteristic->getValue().c_str());
    Serial.println("Característica PV escrita: " + String(value.c_str()));
  }
};
```

#### Si se desea segmentar la información por característica, se crea una clase para cada una de ellas. 

### 6. Inicialización del servidor BLE

#### Finalmente, se inicializa el servidor, se agregan los servicios y se comienza el advertising para ser descubierto.

``` C
// Inicia los servicios BLE
MyService->start();

// Inicia el advertising
pAdvertising = BLEDevice::getAdvertising();
pAdvertising->addServiceUUID(SERVICE_UUID);
pAdvertising->start();
```

## Configuración de ESP32 como master en I2C
#### La ESP32 es configurada como master en I2C para comunicarse a cada TivaC como esclavo. Se emplea la librería Wire.h para manejar este protocolo. 

### 1. Inicialización de la comunicación I2C

#### Los pines de I2C se defienen en la ESP32 como SDA = GPIO 21, SCL = GPIO 22 por defecto. Se establece la velocidad de reloj debido a que el protocolo es síncrono. 
``` C
Wire.begin(); 
Wire.setClock(400000);z
``` 

### 2. Configuración de la comunicación I2C