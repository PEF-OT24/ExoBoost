#include <Wire.h>

#define I2C_DEV_ADDR 0x55

uint32_t i = 0;

void log_print_buf(uint8_t* buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) {
      Serial.print("0");  // Adds a zero for single digit hex values
    }
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.begin();
}

void loop() {
  delay(5000);

  // Write message to the slave
  Wire.beginTransmission(I2C_DEV_ADDR);
  char message[32];
  snprintf(message, sizeof(message), "Hello World! %lu", i++);
  Wire.write((uint8_t*)message, strlen(message));  // Send data as bytes
  uint8_t error = Wire.endTransmission(true);
  Serial.printf("endTransmission: %u\n", error);

  // Read 16 bytes from the slave
  uint8_t bytesReceived = Wire.requestFrom(I2C_DEV_ADDR, 16);

  Serial.printf("requestFrom: %u\n", bytesReceived);
  if (bytesReceived) {
    uint8_t temp[bytesReceived];
    Wire.readBytes(temp, bytesReceived);
    log_print_buf(temp, bytesReceived);
  }
}