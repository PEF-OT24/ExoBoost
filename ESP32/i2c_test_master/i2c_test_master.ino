#include <Wire.h>

#define I2C_SLAVE_ADDR 0x55

void setup() {
  Wire.begin();  // Inicia I2C como máster
  Serial.begin(115200);  // Inicia comunicación serial para debugging
}

void loop() {
  delay(2000);  // Pausa de 2 segundos entre solicitudes
  String message = readI2CMessage(I2C_SLAVE_ADDR);
  Serial.print("Mensaje recibido: ");
  Serial.println(message);  // Imprime el mensaje completo

}

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
  }

  return message;
}
