// ESP32 como master (código que se comunicará con la TivaC)

#include "Wire.h"

#define I2C_DEV_ADDR 0x55

uint32_t i = 0;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.begin();
  // send_data();
}

void loop() {
  // Write message to the slave
  delay(5000);
  Wire.beginTransmission(I2C_DEV_ADDR);
  const char* message = "Aber";
  Wire.printf(message);
  uint8_t error = Wire.endTransmission(true);
  Serial.printf("endTransmission: %u\n", error);

  // // Read 16 bytes from the slave
  // uint8_t bytesReceived = Wire.requestFrom(I2C_DEV_ADDR, 16);
  
  // Serial.printf("requestFrom: %u\n", bytesReceived);
  // if ((bool)bytesReceived) {  //If received more than zero bytes
  //   uint8_t temp[bytesReceived];
  //   Wire.readBytes(temp, bytesReceived);
  //   log_print_buf(temp, bytesReceived);
  // }
  Serial.println("--------------------------------");
}
