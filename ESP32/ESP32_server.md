# Código en la ESP32
#### Es necesario instalar Arduino IDE e instalar el driver para soportar ESP32. 

# Configuración de ESP32 como server BLE

#### La ESP32 genera un servidor de BLE que contiene un determinado número de servicios con características que comunican información con la APP. Para ello son necesarias las siguientes librerias:
``` C
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
```