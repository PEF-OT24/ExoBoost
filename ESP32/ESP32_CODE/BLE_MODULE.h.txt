// Código de declaración de funciones y clases para el módulo de BLE en la ESP32
#ifndef BLE_MODULE_H
#define BLE_MODULE_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>

class MiClase {
public:
    // Constructor
    MiClase();

    // Métodos públicos
    void funcionA();
    void funcionB();
    void funcionC();

private:
    // Variables privadas
    int variablePrivada;
};

#endif