// ESP32 como master (código que se comunicará con la TivaC)

#include "Wire.h"

void setup() {
  // Se inicializa el módulo I2C como mster
  Wire.begin();  // SDA = GPIO 21, SCL = GPIO 22 by default on ESP32

  // Comunicación Serial 
  Serial.begin(115200);

}

void loop() {
  // Main loop code here
  const char* message2 = "ESP32 says Hi!";
  sendI2CMessage(0x55, message2);

  // Se espera 5 segundos
  delay(5000);
}

// Función para mandar un mensaje a través de I2C
void sendI2CMessage(uint8_t slaveAddress, const char* message) {
  int length = strlen(message);  // Calculate the length of the message
  byte byteArray[length];        // Create a byte array of the same length

  // Variables de estatus
  int bytesWritten; // Longitud de los bytes escritos
  int errorCode;    // Código de error después de escribir

  // Se convierte el string a un arreglo de bytes
  for (int i = 0; i < length; i++) {
    byteArray[i] = (byte)message[i];
  }

  // Se imprime el mensaje a mandar
  Serial.print("Sending message: ");
  Serial.println(message);

  // Se manda el mensaje al esclavo I2C
  Wire.beginTransmission(slaveAddress);
  bytesWritten = Wire.write(byteArray, length);  
  errorCode = Wire.endTransmission();

  // Se formatea la salida para debug
  Serial.print("Mandando mensaje a dirección: ");
  Serial.println(slaveAddress);
  Serial.print("Código de error: ");
  Serial.println(errorCode);
  Serial.print("Bytes escritos: ");
  Serial.println(bytesWritten);
  Serial.println("----------------");
}