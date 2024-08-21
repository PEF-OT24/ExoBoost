// Código general para la TIVAC que maneja protocolo CAN e I2C. 
// Se incluye la lógica para el algoritmo de control 

// Importación de librerías
#define PART_TM4C123GH6PM
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h" 
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/can.h"
#include "driverlib/systick.h"
#include <Wire.h>
#include <ArduinoJson.h>

// --------------------------------------- Constantes de uso general -----------------------------------------------
// Definición de pines LED 
#define RED_LED GPIO_PIN_1
#define BLUE_LED GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

// Define CAN_INT_INTID_STATUS if not defined
#ifndef CAN_INT_INTID_STATUS
#define CAN_INT_INTID_STATUS 0x8000
#endif

// Delay entre mensajes de CAN en ms
#define CAN_DELAY 130
// ----------------- Variables globales ------------------
int8_t assitance_level = 0; // Nivel de asistencia
// Parámetros de PI para un determinado motor
uint8_t posKP; 
uint8_t posKI; 
uint8_t velKP; 
uint8_t velKI; 
uint8_t curKP; 
uint8_t curKI; 

uint16_t SP_motor1; // Set Point para el motor 1
uint32_t SP_motor2; // Set Point para el motor 2
uint32_t SP_motor3; // Set Point para el motor 3
uint16_t max_speed = 500; // Velocidad máxima al controlar posición

bool doControlFlag = 0; // Bandera de control en tiempo real 

// ----------------- Variables para I2C ------------------
#define I2C_DEV_ADDR 0x55        // Dirección del esclavo
String mensaje_leido = "";       // Buffer para recibir el mensaje
String mensaje_mandar = "";      // Indicador de qué mensaje se le debe mandar en callback a un request
uint32_t i = 0;
char data;

// ----------------- Variables para CAN ------------------
tCANMsgObject Message_Rx; // Objeto para leer mensajes

// ----------------------------------- Funciones de uso general -----------------------------------
void split32bits(int32_t number, uint8_t *byteArray) {
    // Se divide el número en 4 enteros de 8 bits
    byteArray[0] = (number >> 24) & 0xFF;  // byte más significativo
    byteArray[1] = (number >> 16) & 0xFF;
    byteArray[2] = (number >> 8) & 0xFF;
    byteArray[3] = number & 0xFF;  // byte menos significativo
}

void split16bits(int16_t number, uint8_t *byteArray) {
    // Se divide el número en 2 enteros de 8 bits
    byteArray[0] = (number >> 8) & 0xFF; // byte más significativo  
    byteArray[1] = number & 0xFF;  // byte menos significativo
}

// ----------------------------------- Funciones de interrupción -----------------------------------
void ISRSysTick(void) { // Función de interrupción para tiempo real (NO SE USA)
    doControlFlag = true;
}

void CAN0IntHandler(void) {
    uint8_t CANBUSReceive[8u];
    uint32_t ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    // Check if the interrupt is caused by a status change
    if (ui32Status == CAN_INT_INTID_STATUS) {
        // Read the full status of the CAN controller
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
    } 
    // Check if the interrupt is caused by message object 1
    else if (ui32Status == 1) {
        // Clear the message object interrupt
        CANIntClear(CAN0_BASE, 1);

        // Handle the received message
        Message_Rx.pui8MsgData = CANBUSReceive;
        CANMessageGet(CAN0_BASE, 1, &Message_Rx, true);

        // Example of processing received data
        Serial.print("Mensaje recibido: ");
        for (int i = 0; i < 8; i++) {
            Serial.print(CANBUSReceive[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    } else {
        // Handle unexpected interrupts
        CANIntClear(CAN0_BASE, ui32Status);
    }
    Serial.println("Mensaje recibido");
}
// ----------------------------------- Funciones de manejo de CAN -----------------------------------
void send_cmd(uint8_t ID, uint8_t *messageArray, bool show){ // Función para enviar un mensaje por CAN
  // Objetos para la comunicación CAN
  tCANMsgObject Message_Tx;
  
  uint8_t CAN_data_RX[8u];

  // Define el mensaje para mandar por CAN
  Message_Tx.ui32MsgID = 0x140 + ID;
  Message_Tx.ui32MsgIDMask = 0xFFFFFFFF;
  Message_Tx.ui32MsgLen = 8u;
  Message_Tx.pui8MsgData = messageArray;

  // Define el mensaje para leer por CAN
  Message_Rx.ui32MsgID = 0x240 + ID;
  Message_Rx.ui32MsgIDMask = 0xFFFFFFFF; // Lee todos los mensajes
  Message_Rx.ui32Flags = MSG_OBJ_RX_INT_ENABLE; // Habilita interrupciones en la recepción del mensaje
  Message_Rx.ui32MsgLen = 8u;
  Message_Rx.pui8MsgData = CAN_data_RX;

  // Envío por CAN
  CANMessageSet(CAN0_BASE, ID, &Message_Tx, MSG_OBJ_TYPE_TX); 

  // Lee el mensaje de respuesta
  CANMessageSet(CAN0_BASE, ID, &Message_Rx, MSG_OBJ_TYPE_RX);

  // Imprime el mensaje si el usuario lo indica
  if (show){
    
    char buffer[50];
    while (CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST)) {Serial.println("waiting");}
    sprintf(buffer, "Received: %02X %02X %02X %02X %02X %02X %02X %02X", 
            CAN_data_RX[0], CAN_data_RX[1], CAN_data_RX[2], CAN_data_RX[3], 
            CAN_data_RX[4], CAN_data_RX[5], CAN_data_RX[6], CAN_data_RX[7]);
    Serial.println(buffer);
  }
}

void send_cmd_multiple(char Q, uint8_t *messageArray, bool show){ // Función para enviar un mensaje por CAN
  // Objetos para la comunicación CAN
  tCANMsgObject Message_Tx;

  // Define el mensaje para mandar por CAN
  Message_Tx.ui32MsgID = 0x280;
  Message_Tx.ui32MsgIDMask = 0xFFFFFFFF;
  Message_Tx.ui32MsgLen = 8u;
  Message_Tx.pui8MsgData = messageArray;
  
  // Envío por CAN
  CANMessageSet(CAN0_BASE, 0x280, &Message_Tx, MSG_OBJ_TYPE_TX); 
  
  // Lee el mensaje de respuesta
  for (int i = 1; i <= Q; i++) {
    // Define el mensaje para leer por CAN
    uint8_t CAN_data_RX[8u];
    tCANMsgObject Message_Rx;

    // Se configura el mensaje para cada motor
    Message_Rx.ui32MsgID = 0x240 + i;
    Message_Rx.ui32MsgIDMask = 0xFFFFFFFF; // Lee todos los mensajes
    Message_Rx.ui32MsgLen = 8u;
    Message_Rx.pui8MsgData = CAN_data_RX;
    CANMessageSet(CAN0_BASE, i, &Message_Rx, MSG_OBJ_TYPE_RXTX_REMOTE);
  
    // Imprime el mensaje si el usuario lo indica
    if (show){
      char buffer[50];
      while (CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST)) {Serial.println("waiting");}
      sprintf(buffer, "Received: %02X %02X %02X %02X %02X %02X %02X %02X", 
              CAN_data_RX[0], CAN_data_RX[1], CAN_data_RX[2], CAN_data_RX[3], 
              CAN_data_RX[4], CAN_data_RX[5], CAN_data_RX[6], CAN_data_RX[7]);
      Serial.println(buffer);
    }
  }
}

void SendParameters(uint8_t ID, uint8_t PosKP, uint8_t PosKI, uint8_t SpdKP, uint8_t SpdKI, uint8_t CurrKP, uint8_t CurrKI){
  // Función para enviar los valores de los parámetros PI

  // Se define la variable que almacena el valor
  uint8_t CAN_data_TX[8u];

  // Se definen los valores de los parámetros PI para las tres variables por motor
  CAN_data_TX[0] = 0x32; 
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = CurrKP; // KP para corriente
  CAN_data_TX[3] = CurrKI; // KI para corriente
  CAN_data_TX[4] = SpdKP;  // KP para velocidad
  CAN_data_TX[5] = SpdKI;  // KI para velocidad
  CAN_data_TX[6] = PosKP;  // KP para posición
  CAN_data_TX[7] = PosKI;  // KI para posición
  
  send_cmd(ID, CAN_data_TX, true);
}

void ReadParameters(int8_t ID, bool show){ // MEJORA: RETORNAR PARÁMETROS LEÍDOS
  // Función para leer los parámetros PI

  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];
  
  // Define el setpoint de la velocidad
  CAN_data_TX[0] = 0x30;
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = 0x00;
  CAN_data_TX[5] = 0x00;
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, show);
}

void set_speed(int8_t ID, int64_t speed_ref, bool show){
  // Función para establecer la velocidad
  
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Escala el valor a enviar 0.01 dps/LSB
  int32_t sp = speed_ref * 100;

  uint8_t byteArray[4];
  split32bits(sp, byteArray);
  
  // Define el setpoint de la velocidad
  CAN_data_TX[0] = 0xA2;
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = byteArray[3];
  CAN_data_TX[5] = byteArray[2];
  CAN_data_TX[6] = byteArray[1];
  CAN_data_TX[7] = byteArray[0];

  send_cmd(ID, CAN_data_TX, show);
}

void read_acceleration(int8_t ID, bool show){
  // Función para leer aceleración 
  
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];
  
  // Define el setpoint de la velocidad
  CAN_data_TX[0] = 0x42;
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = 0x00;
  CAN_data_TX[5] = 0x00;
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, show);
}

void set_acceleration(int8_t ID, int64_t accel_ref, bool show){
  // Función para establecer la aceleración
  
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Escala el valor a enviar 0.01 dps/LSB
  int32_t sp = accel_ref * 100;

  uint8_t byteArray[4];
  split32bits(sp, byteArray);
  
  // Define el setpoint de la velocidad
  CAN_data_TX[0] = 0x43;
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = byteArray[3];
  CAN_data_TX[5] = byteArray[2];
  CAN_data_TX[6] = byteArray[1];
  CAN_data_TX[7] = byteArray[0];

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, show);
}

void set_absolute_position(int8_t ID, int32_t position_ref, int16_t max_speed, bool show){ // NO FUNCIONA
  // Función para establecer la posición absoluta
  
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Escala el valor a enviar 0.01 dps/LSB
  int32_t sp = position_ref * 100;

  uint8_t byteArray_speed[2];
  uint8_t byteArray_position[4];

  // Se dividen los valores en 2 y 4 bytes
  split32bits(sp, byteArray_position);
  split16bits(max_speed, byteArray_speed);
  
  // Define el setpoint de la velocidad
  CAN_data_TX[0] = 0xA4;
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = byteArray_speed[1];
  CAN_data_TX[3] = byteArray_speed[0];
  CAN_data_TX[4] = byteArray_position[3];
  CAN_data_TX[5] = byteArray_position[2];
  CAN_data_TX[6] = byteArray_position[1];
  CAN_data_TX[7] = byteArray_position[0];

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, show);
}

void set_incremental_position(int8_t ID, int32_t position_inc, int16_t max_speed, bool show){ 
  // Función para establecer la posición incremental
  
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Escala el valor a enviar 0.01 dps/LSB
  int32_t sp = position_inc * 100;

  uint8_t byteArray_speed[2];
  uint8_t byteArray_position[4];

  // Split the number into 4 bytes
  split32bits(sp, byteArray_position);
  split16bits(max_speed, byteArray_speed);
  
  // Define el setpoint de la velocidad
  CAN_data_TX[0] = 0xA8;
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = byteArray_speed[1];
  CAN_data_TX[3] = byteArray_speed[0];
  CAN_data_TX[4] = byteArray_position[3];
  CAN_data_TX[5] = byteArray_position[2];
  CAN_data_TX[6] = byteArray_position[1];
  CAN_data_TX[7] = byteArray_position[0];

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, show);
}

void set_stposition(int8_t ID, int16_t position_inc, int16_t max_speed, int8_t direction, bool show){ 
  // Función para establecer la posición en relativo a una vuelta
  
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Escala el valor a enviar 0.01 dps/LSB
  int32_t sp = position_inc * 100;

  uint8_t byteArray_speed[2];
  uint8_t byteArray_position[2];

  // Split the number into 4 bytes
  split16bits(sp, byteArray_position);
  split16bits(max_speed, byteArray_speed);
  
  // Define el setpoint de la velocidad
  CAN_data_TX[0] = 0xA6;
  CAN_data_TX[1] = direction;
  CAN_data_TX[2] = 0x02;
  CAN_data_TX[3] = 0x02;
  CAN_data_TX[4] = byteArray_position[1];
  CAN_data_TX[5] = byteArray_position[0];
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, show);
}

void set_torque(int8_t ID, int64_t current_torque, bool show){ 
  // Función para establecer el torque a través de la corriente
  
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  int32_t sp = current_torque * 100;

  uint8_t byteArray_current[2];

  // Split the number into 4 bytes
  split16bits(sp, byteArray_current);
  
  // Define el setpoint de la velocidad
  CAN_data_TX[0] = 0xA1;
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = byteArray_current[1];
  CAN_data_TX[5] = byteArray_current[0];
  CAN_data_TX[6] = 0;
  CAN_data_TX[7] = 0;

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, show);
}

void stop_motor(int8_t ID, bool show){
  // Función para detener el motor
  
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Reset del motor
  CAN_data_TX[0] = 0x81; 
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = 0x00;
  CAN_data_TX[5] = 0x00;
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, show);
}

void stop_all_motors(bool show){
  // Función para detener el motor
  
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Reset del motor
  CAN_data_TX[0] = 0x81; 
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = 0x00;
  CAN_data_TX[5] = 0x00;
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

  // Se envía el mensaje
  send_cmd_multiple(2, CAN_data_TX, show);
}

void shutdown_motor(int8_t ID, bool show){
  // Función para apagar el motor
  
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Reset del motor
  CAN_data_TX[0] = 0x80; 
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = 0x00;
  CAN_data_TX[5] = 0x00;
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, show);
}

void reset_motor(int8_t ID){
  // Función para restablecer los comandos enviados al motor
  
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Reset del motor
  CAN_data_TX[0] = 0x76; 
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = 0x00;
  CAN_data_TX[5] = 0x00;
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, true);
}

void reset_all_motors(bool show){
  // Función para mandar un reset a todos los motores presentes en la RED CAN
  
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Reset del motor
  CAN_data_TX[0] = 0x76; 
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = 0x00;
  CAN_data_TX[5] = 0x00;
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

  // Objetos para la comunicación CAN
  tCANMsgObject Message_Tx;

  // Define el mensaje para mandar por CAN
  Message_Tx.ui32MsgID = 0x280;
  Message_Tx.ui32MsgIDMask = 0xFFFFFFFF;
  Message_Tx.ui32MsgLen = 8u;
  Message_Tx.pui8MsgData = CAN_data_TX;
  
  // Envío por CAN
  CANMessageSet(CAN0_BASE, 0x280, &Message_Tx, MSG_OBJ_TYPE_TX); 
}

// ----------------------------------- Funciones de callback de manejo de I2c -----------------------------------
void onReceive(int len){
  // Función de callback que se ejecuta al recibir un mensaje por I2C

  // Lee los bytes recibidos
  while (Wire.available()) { // Guarda los bytes recibidos
    char rec_data = Wire.read();
    mensaje_leido += rec_data;
  }

  if (mensaje_leido.endsWith("\n")){ // Indicador de que el mensaje está completo
    mensaje_leido.trim();
    JsonDocument jsonrec;            // Archivo json para recibir información 
    
    Serial.print("Datos recibidos: ");
    Serial.println(mensaje_leido);
  
    // ----- Procesamiento del mensaje recibido -----
    const char *value = mensaje_leido.c_str();
    DeserializationError error = deserializeJson(jsonrec, value);
  
    // Se revisan errores
    if (error) { // Error al deserializar
      Serial.print("Error al analizar JSON: ");
      Serial.println(error.c_str());
      return;
    }
    else if (!jsonrec.containsKey("T")){ // Si no contiene el tipo
      Serial.println("Tipo no encontrado en el JSON");
      return;
    }
    else if (!jsonrec["T"].is<const char*>()){
      Serial.println("Tipo de dato erróneo");
      return;  
    }
  
    // ----------- Se procesan diferentes JSON --------
    const char* type = jsonrec["T"];
    if (strcmp(type, "F") == 0){ // Se establece el tipo de mensaje a mandar
      mensaje_mandar = "PV";
    }
    else if (strcmp(type, "A") == 0){ 
      // ------------- Nivel de asistencia -------------
      // Ejemplo: {"T": "A","asistance_level": "100"}

      // Se revisan errores en el JSON
      if (!jsonrec.containsKey("asistance_level")){ // Si si no contiene el tipo
        Serial.print("Información no encontrada");
        return;
      }
      else if (!jsonrec["asistance_level"].is<String>()){
        Serial.println("Información en formato incorrcto");
        return;
      }
  
      // Se extrae la información recibida
      assitance_level = jsonrec["asistance_level"].as<int>(); // Se guarda el valor
      Serial.print("Nivel de asistencia: ");
      Serial.println(assitance_level);
    }
    else if (strcmp(type, "B") == 0 || strcmp(type, "C") == 0 || strcmp(type, "D") == 0){ // Parámetros PI para cualquier motor
      // ------------- Parámetros de PI -------------
      /*Ejemplo
      {
        "T": "B",
        "motor1":{
          "pos": {"kc": "100", "ti": "50"},
          "vel": {"kc": "100", "ti": "50"},
          "cur": {"kc": "100", "ti": "50"},
        }      
      }    
      */   

      char address = 0; // dirección del motor
      JsonDocument json_motor;
      if (jsonrec.containsKey("motor1")){ // Si contiene el motor 1
        Serial.println("Parámetros del motor 1");
        address = 1;
        json_motor = jsonrec["motor1"];
      }  
      else if (jsonrec.containsKey("motor2")){
        Serial.println("Parámetros del motor 2");
        address = 2; 
        json_motor = jsonrec["motor2"];
      }
      else if (jsonrec.containsKey("motor3")){
        Serial.println("Parámetros del motor 3");
        address = 3;
        json_motor = jsonrec["motor3"];
      }
      else {
        Serial.print("Información no encontrada");
        return;  
      }

      // Revisión de errores dentro del nuevo archivo JSON
      if (!json_motor.containsKey("pos") || !json_motor.containsKey("vel") || !json_motor.containsKey("cur")){
        Serial.print("Error en los datos encontrados");
        return;
      }
      
      JsonDocument json_parametros;
      // Procesamiento para parámetros de posición
      json_parametros = json_motor["pos"];
      if (!json_parametros.containsKey("kc") || !json_parametros.containsKey("ti")){
        Serial.println("Parámetros no encontrados");
        return;
      }
      posKP = json_parametros["kc"].as<int>();
      posKI = json_parametros["ti"].as<int>();
      json_parametros.clear();

      // Procesamiento para parámetros de velocidad
      json_parametros = json_motor["vel"];
      if (!json_parametros.containsKey("kc") || !json_parametros.containsKey("ti")){
        Serial.println("Parámetros no encontrados");
        return;
      }
      velKP = json_parametros["kc"].as<int>();
      velKI = json_parametros["ti"].as<int>();
      json_parametros.clear();
      
      // Procesamiento para parámetros de corriente
      json_parametros = json_motor["cur"];
      if (!json_parametros.containsKey("kc") || !json_parametros.containsKey("ti")){
        Serial.println("Parámetros no encontrados");
        return;
      }
      curKP = json_parametros["kc"].as<int>();
      curKI = json_parametros["ti"].as<int>();
      json_parametros.clear();

      // Se mandan los parámetros
      Serial.println("Parámetros de posición: ");
      Serial.print("kc: "); 
      Serial.println(posKP);
      Serial.print("ti: ");
      Serial.println(posKI);
      Serial.println("Parámetros de velocidad: ");
      Serial.print("kc: "); 
      Serial.println(velKP);
      Serial.print("ti: ");
      Serial.println(velKI);
      Serial.println("Parámetros de corriente: ");
      Serial.print("kc: "); 
      Serial.println(curKP);
      Serial.print("ti: ");
      Serial.println(curKI);
      SendParameters(address, posKP, posKI, velKP, velKI, curKP, curKI);
    }
    else if(strcmp(type, "E") == 0) {
      // ------- Mandar el Set Point -------
      /* Ejemplo
        {
          "T": "E",
          "monitoring": "pos", 
          "motor1": "0",
          "motor2": "0",
          "motor3": "0",
        }
      */
      // Se revisan errores
      if (!jsonrec.containsKey("monitoring") || !jsonrec.containsKey("motor1") || !jsonrec.containsKey("motor2") || !jsonrec.containsKey("motor3")){
        Serial.print("Información no encontrada");
        return;
      }

      // Se extraen los Set Points del mensaje
      SP_motor1 = jsonrec["motor1"].as<int>();
      SP_motor2 = jsonrec["motor2"].as<int>();
      SP_motor3 = jsonrec["motor3"].as<int>();
      
      // Control dependiendo del tipo 
      const char* process_variable = jsonrec["monitoring"];
      if (strcmp(process_variable, "pos") == 0){
        // Control de posición
        Serial.println("Control de posición");
        set_absolute_position(1, SP_motor1, max_speed, true);
        delay(CAN_DELAY); // delay 
        set_absolute_position(2, SP_motor2, max_speed, true);
        delay(CAN_DELAY); // delay 
        //set_absolute_position(3, SP_motor3, max_speed, true);
        //delay(CAN_DELAY); // delay 
      }
      else if(strcmp(process_variable, "vel") == 0){
        // Control de velocidad
        Serial.println("Control de velocidad");
        set_speed(1, SP_motor1, true);
        delay(CAN_DELAY); // delay 
        set_speed(2, SP_motor2, true);
        delay(CAN_DELAY); // delay 
        //set_speed(3, SP_motor3, true);
        //delay(CAN_DELAY); // delay 
      }
      else if (strcmp(process_variable, "cur") == 0){
        // Control de torque
        Serial.println("Control de torque");
        set_torque(1, SP_motor1, false);
        delay(CAN_DELAY); // delay 
        set_torque(2, SP_motor2, false);
        delay(CAN_DELAY); // delay 
        //set_torque(3, SP_motor3, false);
        //delay(CAN_DELAY); // delay 
      }
    }
    else if (strcmp(type, "H") == 0){
      // ------- Comandos a los motores -------
      /* Ejemplo
        {"state": "stop", "T", "H"}
      */
      if (!jsonrec.containsKey("state")){
        Serial.println("Información no disponible");
        return;  
      }
      const char* state_command = jsonrec["state"];
      if (strcmp(state_command, "stop") == 0){ // Comando de detenerse
        Serial.println("Deteniendo motores");
        stop_motor(1, false);
        delay(CAN_DELAY);
        stop_motor(2, false);
        delay(CAN_DELAY);
        stop_motor(3, false);
        delay(CAN_DELAY);
      }
    }
  
    // Se limpia el buffer y el archivo json receptor
    mensaje_leido = "";
  }
}

void onRequest(){
  // Función de callback que se ejecuta al recibir un request por I2C
  // Se debe de recibir primero el tipo de mensaje que se quiere mandar

  // Se define el mensaje a mandar
  if(mensaje_mandar == "PV"){
    Serial.println("test");
  }

  for (int i = 0; i < 100; i++){
    Wire.write("a"); // test de respuesta  
  }

  // AGREGAR LÓGICA PARA MANDAR EL JSON
}

// ----------------------------------------------------- Setup ----------------------------------------------------
void setup() {
    Serial.begin(9600);

    // Enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED);

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    ADCSequenceEnable(ADC0_BASE, 0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0)) {}
    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 1000000u);
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    CANEnable(CAN0_BASE);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    SysTickIntRegister(ISRSysTick);
    SysTickPeriodSet(11200);
    SysTickIntEnable();
    SysTickEnable();

    // ------ Configuración de I2C ------
    Wire.begin((uint8_t)I2C_DEV_ADDR);                // Inicializa el protocolo I2C
    Wire.onReceive(onReceive); // Se registra el evento de onreceive
    Wire.onRequest(onRequest); // register event

    IntMasterEnable();
    //CANIntRegister(CAN0_BASE,CAN0IntHandler);
    //SendParameters();
    //stop_motor(1);
    //reset_motor(1);
    //shutdown_motor(1);
    //set_acceleration(1,500,true);
    //set_incremental_position(1, 90, 360, true);
    //set_speed(1, 360, true);
    //set_absolute_position(1, 90, 1000, true);
    //set_stposition(1,90,1000,0,true);

    Serial.println("Listo: ");
}

// ----- Main Loop -----
void loop() {
  //set_stposition(1,90,5000,1,true);
  //set_speed(1, 360, true);
  //stop_motor(1);
  //reset_motor(1);
}
