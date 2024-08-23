#include <Wire.h>

#define I2C_SLAVE_ADDR 0x55
#define BUFFER_SIZE 32

String message = "";
int indice = 0;
bool message_defined = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SLAVE_ADDR);  // Inicia I2C como esclavo
  Wire.onRequest(requestEvent);  // Configura función a ejecutar en solicitud
}

void loop() {
  // Lógica adicional si se necesita, se puede manejar aquí.
}

void requestEvent() {
  if (!message_defined) {
    // Aquí defines el mensaje dinámicamente. Este es solo un ejemplo.
    message = "Este es un mensaje dinámico que puede cambiar en cada solicitud.\n";
    message_defined = true;  // Marca que el mensaje ya fue definido
  }

  int bytesToSend = min(BUFFER_SIZE, message.length() - indice);
  Serial.print(message.substring(indice, indice + bytesToSend).c_str());
  Wire.write(message.substring(indice, indice + bytesToSend).c_str(), bytesToSend);
  indice += bytesToSend;

  if (indice >= message.length()) {
    indice = 0;           // Reinicia el índice para el próximo ciclo
    message_defined = false;  // Permite redefinir el mensaje en la próxima solicitud
  }
}
