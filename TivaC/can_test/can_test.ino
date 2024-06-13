// This sketch prints "Hello, world!" to the serial port every second.

void setup() {
  // Start the serial communication at 9600 baud rate:
  Serial.begin(9600);
}

void loop() {
  // Print a message to the serial monitor:
  Serial.println("Hello, world!");

  // Wait for a second before sending the next message:
  delay(1000);
}
