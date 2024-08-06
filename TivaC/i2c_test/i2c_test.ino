#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include <Wire.h>

#define I2C_DEV_ADDR 0x55

uint32_t i = 0;

void onRequest() {
  char message[32];
  snprintf(message, sizeof(message), "%lu Packets.", i++);
  Wire.write((uint8_t*)message, strlen(message));  // Send data as bytes
  Serial.println("onRequest");
  Serial.println(message);
}

void onReceive(int len) {
  Serial.print("onReceived: " + len);
  while (Wire.available()) {
    Serial.write(Wire.read());
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  //Serial.setDebugOutput(true);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
}

void loop() {
  // No need for anything in the loop
}