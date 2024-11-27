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
pServer = BLEDevice::createServer();
pServer->setCallbacks(new ServerCallbacks());
```

#### En el campo <DEVICE_NAME> se le asigna el nombre al dispositivo para ser descubierto; este atributo forma parte del descriptor GATT.

### 2. Asignar una clase de callbacks para el servidor

#### BLE en la ESP32 funciona a base de métodos callbacks, es decir, interrupciones cuando algún evento de interés sucede. Para configurar las interrupciones del servidor, se crea la siguiente clase personalizable:

``` C
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { 
    // Método que se ejecuta en conexión
    Serial.println("Cliente CONECTADO");
    client_connected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    // Método que se ejecuta en conexión
    client_connected = false;
    Serial.println("Cliente desconectado.");
  }
};
```

### 3. Crear el servicio

#### Se crea una isntancia de la clase BLEService para cada servicio.
#### Toma como argumento su UUID <SERVICE_UUID_PARAMS>

``` C
BLEService *pService_PARAMS = pServer->createService(SERVICE_UUID_PARAMS);
```

### 4. Crear las características
#### Para cada servicio, se crean sus características

``` C
pCharacteristic_PI = pService_PARAMS->createCharacteristic(
                                        CHARACTERISTIC_UUID_PI,
                                        BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_WRITE |
                                        BLECharacteristic::PROPERTY_NOTIFY |
                                        BLECharacteristic::PROPERTY_INDICATE
                                    );
pCharacteristic_PI->setCallbacks(new BLECallback_PI());
```