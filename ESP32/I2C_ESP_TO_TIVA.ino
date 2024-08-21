#include <Wire.h>

#define SLAVE_ADDRESS 0x04

void setup() {
  Wire.begin(); // Inicializa el I2C como maestro
  Serial.begin(115200);
}

void loop() {
  // Enviar señal para encender el LED
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(0x01); // Envía el mensaje para encender el LED en el esclavo
  if (Wire.endTransmission() == 0) {
    Serial.println("LED ON signal sent to Tiva C");
  } else {
    Serial.println("Error sending LED ON signal");
  }
  delay(500);

  // Enviar señal para apagar el LED
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write(0x00); // Envía el mensaje para apagar el LED en el esclavo
  if (Wire.endTransmission() == 0) {
    Serial.println("LED OFF signal sent to Tiva C");
  } else {
    Serial.println("Error sending LED OFF signal");
  }
  delay(500);
}
