// Código para configurar una TivaC como esclavo en una red 
#include <Wire.h>

#define I2C_DEV_ADDR 0x55 // Dirección del esclavo
uint32_t i = 0;

void setup(){
  Wire.begin((uint8_t)I2C_DEV_ADDR);                // Inicializa el protocolo I2C 
  Wire.onReceive(onReceive); // Se registra el evento de onreceive
  Serial.begin(9600);           // start serial for output
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void onReceive(int len){
  Serial.print("onReceive " + len);
  while (Wire.available()) {
    Serial.write(Wire.read());
  }
  Serial.println();
}

void loop(){
  // Main loop vacío
  Serial.println("test");
  delay(500);
}
