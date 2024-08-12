// Incluye la librería para la comunicación serial
#include <HardwareSerial.h>

// Define los pines UART para la comunicación
#define UART_TX_PIN 17 // Pin de transmisión (TX)
#define UART_RX_PIN 16 // Pin de recepción (RX)

// Crea una instancia del puerto serial
HardwareSerial mySerial(1);

void setup() {
  // Inicializa el monitor serial a 115200 baudios
  Serial.begin(115200);
  // Inicializa el puerto serial UART con los pines definidos y una velocidad de 9600 baudios
  mySerial.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // Mensaje de depuración en el monitor serial
  Serial.println("ESP32 inicializado.");
  Serial.println("Configurando UART...");
}

void loop() {
  // Envía un mensaje a través del UART
  mySerial.println("A");

  // Mensaje de depuración en el monitor serial
  Serial.println("Mensaje enviado a través de UART.");

  // Espera 1 segundo antes de enviar el siguiente mensaje
  delay(1000);
}
