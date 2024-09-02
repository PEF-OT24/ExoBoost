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
int8_t assistance_level = 0; // Nivel de asistencia
// Parámetros de PI para un determinado motor
uint8_t posKP; 
uint8_t posKI; 
uint8_t velKP; 
uint8_t velKI; 
uint8_t curKP; 
uint8_t curKI; 

// Parámetros de Set Point para los motores
uint16_t SP_motor1;
uint32_t SP_motor2;
uint32_t SP_motor3;
uint16_t max_speed = 500; // Velocidad máxima al controlar posición

// Variables de proceso para los motores
String process_variable = "vel"; // Tipo de variable de proceso: pos, vel, cur, temp
int32_t PV1 = 1;
int32_t PV2 = 1;
int32_t PV3 = 999;

bool doControlFlag = 0; // Bandera de control en tiempo real 
// ----------------- Variables para I2C ------------------
#define I2C_DEV_ADDR 0x55        // Dirección del esclavo
const int BUFFER_SIZE = 32;      // Tamaño máximo del buffer
String mensaje_leido = "";       // Buffer para recibir el mensaje
byte commandCAN = 0x00;          // Guarda el comando mandado para lectura de comandos de CAN

// Variables para la transmisión de mensajes por I2C
String stringsend_ESP32 = "";   // Se inicializa el mensaje como vacío
JsonDocument jsonsend_ESP32;
int index_alt;

// ----------------- Variables para CAN ------------------
// lectura de CAN de múltiples motores
tCANMsgObject Message_Rx_1;   // Objeto para leer mensajes del motor 1
tCANMsgObject Message_Rx_2;   // Objeto para leer mensajes del motor 2
tCANMsgObject Message_Rx_3;   // Objeto para leer mensajes del motor 3
uint8_t motor_selected = 0; // Indicador del motor seleccionado para lectura

// ----------------------------------- Funciones de uso general -----------------------------------
void split32bits(int32_t number, uint8_t *byteArray) {              // Función para dividir una variable de 32 bits en 4 bytes
  // Se divide el número en 4 enteros de 8 bits
  byteArray[0] = (number >> 24) & 0xFF;  // byte más significativo
  byteArray[1] = (number >> 16) & 0xFF;
  byteArray[2] = (number >> 8) & 0xFF;
  byteArray[3] = number & 0xFF;  // byte menos significativo
}

void split16bits(int16_t number, uint8_t *byteArray) {            // Función para dividir una variable de 16 bits en 2 bytes
    // Se divide el número en 2 enteros de 8 bits
    byteArray[0] = (number >> 8) & 0xFF; // byte más significativo  
    byteArray[1] = number & 0xFF;  // byte menos significativo
}

void delayMS(uint32_t milliseconds){                            // Función que ejecuta un delay de manera aproximada (las interrupciones tienen prioridad)
    uint32_t delay_cycles;
    
    // Get the system clock frequency in Hz
    uint32_t sysClock = SysCtlClockGet();

    // Calculate the number of delay cycles needed for the given milliseconds
    // Avoid overflow by dividing first
    delay_cycles = (sysClock / 3000) * milliseconds;

    // Call SysCtlDelay to create the delay
    SysCtlDelay(delay_cycles);
}
// ----------------------------------- Funciones de interrupción -----------------------------------
void ISRSysTick(void) { // Función de interrupción para tiempo real
    doControlFlag = true;
}

void CAN0IntHandler(void) { // Función de interrupción para recepción de mensajes de CAN
    uint8_t CANBUSReceive[8u];
    uint32_t ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
    
    // Check if the interrupt is caused by a status change
    if (ui32Status == CAN_INT_INTID_STATUS) {
        // Read the full status of the CAN controller
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        // ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);
        
    } 
    // Check if the interrupt is caused by message object 1
    else if (ui32Status == 1 || ui32Status == 2 || ui32Status == 3) { // motores
        // Clear the message object interrupt
        CANIntClear(CAN0_BASE, ui32Status);        // limpia la fuente del interrupt

        // procedimiento diferente dependiendo del motor leido
        if (motor_selected == 1){                  // motor 1 
          // Se obtiene el mensaje
          Message_Rx_1.pui8MsgData = CANBUSReceive;
          CANMessageGet(CAN0_BASE, 1, &Message_Rx_1, false);
  
          // Validación del mensaje
          if (commandCAN != CANBUSReceive[0]){
            return;
          }

          // Se formatea la información dependiendo del comando recibido
          if (process_variable == "pos"){ // Formateo para posición
            PV1 = int(round(((CANBUSReceive[7] << 24) | (CANBUSReceive[6] << 16) | (CANBUSReceive[5] << 8) | CANBUSReceive[4])/100));  
          } else if (process_variable == "vel"){ // Formateo para velocidad
            int16_t PV1_read;
            PV1_read = int(round((CANBUSReceive[5] << 8) | CANBUSReceive[4]));  
            PV1 = PV1_read;
          } else if (process_variable == "cur"){ // Formateo para corriente
            int16_t PV1_read;
            PV1_read = int(round(((CANBUSReceive[3] << 8) | CANBUSReceive[2]))/100);
            PV1 = PV1_read;
          }

        } else if (motor_selected == 2){           // motor 2
          // Se obtiene el mensaje
          Message_Rx_2.pui8MsgData = CANBUSReceive;
          CANMessageGet(CAN0_BASE, 2, &Message_Rx_2, false);
  
          // Validación del mensaje
          if (commandCAN != CANBUSReceive[0]){
            return;
          }
          
          // Se formatea la información dependiendo del comando recibido
          if (process_variable == "pos"){ // Formateo para posición
            PV2 = int(round(((CANBUSReceive[7] << 24) | (CANBUSReceive[6] << 16) | (CANBUSReceive[5] << 8) | CANBUSReceive[4])/100));  
          } else if (process_variable == "vel"){ // Formateo para velocidad
            int16_t PV2_read;
            PV2_read = int(round((CANBUSReceive[5] << 8) | CANBUSReceive[4]));  
            PV2 = PV2_read;
          } else if (process_variable == "cur"){ // Formateo para corriente
            int16_t PV2_read;
            PV2_read = int(round(((CANBUSReceive[3] << 8) | CANBUSReceive[2]))/100);
            PV2 = PV2_read;
          }
          
        } else if (motor_selected == 3){           // motor 3
          // Se obtiene el mensaje
          Message_Rx_3.pui8MsgData = CANBUSReceive;
          CANMessageGet(CAN0_BASE, 3, &Message_Rx_3, false);
  
          // Validación del mensaje
          if (commandCAN != CANBUSReceive[0]){
            return;
          }
          
          // Se formatea la información dependiendo del comando recibido
          if (process_variable == "pos"){ // Formateo para posición
            PV3 = int(round(((CANBUSReceive[7] << 24) | (CANBUSReceive[6] << 16) | (CANBUSReceive[5] << 8) | CANBUSReceive[4])/100));  
          } else if (process_variable == "vel"){ // Formateo para velocidad
            int16_t PV3_read;
            PV3_read = int(round((CANBUSReceive[5] << 8) | CANBUSReceive[4]));  
            PV3 = PV3_read;
          } else if (process_variable == "cur"){ // Formateo para corriente
            int16_t PV3_read;
            PV3_read = int(round(((CANBUSReceive[3] << 8) | CANBUSReceive[2]))/100);
            PV3 = PV3_read;
          }
        } else {/*Serial.println("Error")*/;}             // error en el motor seleccionado

    } else {
        // Handle unexpected interrupts
        CANIntClear(CAN0_BASE, ui32Status);
    }
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
  Message_Tx.ui32Flags = MSG_OBJ_TX_INT_ENABLE; // Habilita interrupciones en el envío de mensaje
  Message_Tx.pui8MsgData = messageArray;

  if (ID == 1){       // Establecimiento de mensajes para el motor 1
    // Define el mensaje para leer por CAN
    Message_Rx_1.ui32MsgID = 0x241;
    Message_Rx_1.ui32MsgIDMask = 0xFFFFFFFF; // Lee todos los mensajes
    Message_Rx_1.ui32Flags = MSG_OBJ_RX_INT_ENABLE; // Habilita interrupciones en la recepción del mensaje
    Message_Rx_1.ui32MsgLen = 8u;
    Message_Rx_1.pui8MsgData = CAN_data_RX;
  } else if (ID == 2){ // Establecimiento de mensajes para el motor 2
    // Define el mensaje para leer por CAN
    Message_Rx_2.ui32MsgID = 0x242;
    Message_Rx_2.ui32MsgIDMask = 0xFFFFFFFF; 
    Message_Rx_2.ui32Flags = MSG_OBJ_RX_INT_ENABLE; 
    Message_Rx_2.ui32MsgLen = 8u;
    Message_Rx_2.pui8MsgData = CAN_data_RX;
  } else if (ID == 3){ // Establecimiento de mensajes para el motor 3
    // Define el mensaje para leer por CAN
    Message_Rx_3.ui32MsgID = 0x243;
    Message_Rx_3.ui32MsgIDMask = 0xFFFFFFFF; 
    Message_Rx_3.ui32Flags = MSG_OBJ_RX_INT_ENABLE; 
    Message_Rx_3.ui32MsgLen = 8u;
    Message_Rx_3.pui8MsgData = CAN_data_RX;
  }

  // Envío por CAN
  CANMessageSet(CAN0_BASE, ID, &Message_Tx, MSG_OBJ_TYPE_TX);

  // Configura los objetos de mensajes
  if (ID == 1){
    CANMessageSet(CAN0_BASE, 1, &Message_Rx_1, MSG_OBJ_TYPE_RX);
  } else if (ID == 2){
    CANMessageSet(CAN0_BASE, 2, &Message_Rx_2, MSG_OBJ_TYPE_RX);
  } else if (ID == 3){
    CANMessageSet(CAN0_BASE, 3, &Message_Rx_3, MSG_OBJ_TYPE_RX);
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

  int16_t sp = current_torque * 100;

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
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

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
  send_cmd(ID, CAN_data_TX, false);
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
  // send_cmd_multiple(2, CAN_data_TX, show);
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
  send_cmd(ID, CAN_data_TX, false);
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

void read_angle(int8_t ID){
  // Función para apagar el motor

  motor_selected = ID;
  commandCAN = 0x92; // comando de lectura
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Reset del motor
  CAN_data_TX[0] = 0x92;
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = 0x00;
  CAN_data_TX[5] = 0x00;
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, false);
}

void read_velocity(int8_t ID){
  // Función para apagar el motor

  motor_selected = ID;
  commandCAN = 0x9C; // comando de lectura de velocidad (también hay otros parámetros)
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Reset del motor
  CAN_data_TX[0] = 0x9C;
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = 0x00;
  CAN_data_TX[5] = 0x00;
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, false);
}

void read_current(int8_t ID){
  // Función para apagar el motor

  motor_selected = ID;
  commandCAN = 0x9C; // comando de lectura de velocidad (también hay otros parámetros)
  // Objetos para la comunicación CAN
  uint8_t CAN_data_TX[8u];

  // Reset del motor
  CAN_data_TX[0] = 0x9C;
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = 0x00;
  CAN_data_TX[5] = 0x00;
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

  // Se envía el mensaje
  send_cmd(ID, CAN_data_TX, false);
}

// ----------------------------------- Funciones de callback de manejo de I2c -----------------------------------
// Función para reiniciar el bus I2C ante algún error
void clearI2C() {
  while (Wire.available()) {
    Wire.read();  // Lee y desecha cualquier dato restante en el buffer
  }
  mensaje_leido = "";
}

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
  
    // ----- Procesamiento del mensaje recibido -----
    const char *value = mensaje_leido.c_str();
    DeserializationError error = deserializeJson(jsonrec, value);
  
    // Se revisan errores
    if (error) { // Error al deserializar
      Serial.print("Error al analizar JSON: ");
      Serial.println(error.c_str());
      clearI2C();
      return;
    }
    else if (!jsonrec.containsKey("T")){ // Si no contiene el tipo
      //Serial.println("Tipo no encontrado en el JSON");
      clearI2C();
      return;
    }
    else if (!jsonrec["T"].is<const char*>()){
      //Serial.println("Tipo de dato erróneo");
      clearI2C();
      return;  
    }
  
    // ----------- Se procesan diferentes JSON --------
    const char* type = jsonrec["T"];
    if (strcmp(type, "F") == 0){ // Se establece el tipo de mensaje a mandar
      
      jsonsend_ESP32.clear();    // Se limpia el JSON anterior
      stringsend_ESP32 = "";     // Se reinicia el string anterior

      // Se define el mensaje con los valores de la variable de proceso
      jsonsend_ESP32["monitoring"] = process_variable;
      jsonsend_ESP32["limb"] = "Right leg";
      jsonsend_ESP32["motor1"] = String(PV1);
      jsonsend_ESP32["motor2"] = String(PV2);
      jsonsend_ESP32["motor3"] = String(PV3);

      // Se construye el mensaje serializado
      serializeJson(jsonsend_ESP32, stringsend_ESP32);
      stringsend_ESP32 += '\n';  // terminador '\n'

      //Serial.print("String ESP32: ");
      //Serial.println(stringsend_ESP32);
      
    }
    else if (strcmp(type, "A") == 0){ 
      // ------------- Nivel de asistencia -------------
      // Ejemplo: {"T": "A","assistance_level": "100"}

      // Se revisan errores en el JSON
      if (!jsonrec.containsKey("assistance_level")){ // Si si no contiene el tipo
        //Serial.print("Información no encontrada");
        clearI2C();
        return;
      }
      else if (!jsonrec["assistance_level"].is<String>()){
        //Serial.println("Información en formato incorrcto");
        clearI2C();
        return;
      }
  
      // Se extrae la información recibida
      assistance_level = jsonrec["assistance_level"].as<int>(); // Se guarda el valor
      //Serial.print("Nivel de asistencia: ");
      //Serial.println(assistance_level);
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
        //Serial.println("Parámetros del motor 1");
        address = 1;
        json_motor = jsonrec["motor1"];
      }  
      else if (jsonrec.containsKey("motor2")){
        //Serial.println("Parámetros del motor 2");
        address = 2; 
        json_motor = jsonrec["motor2"];
      }
      else if (jsonrec.containsKey("motor3")){
        //Serial.println("Parámetros del motor 3");
        address = 3;
        json_motor = jsonrec["motor3"];
      }
      else {
        //Serial.print("Información no encontrada");
        clearI2C();
        return;  
      }

      // Revisión de errores dentro del nuevo archivo JSON
      if (!json_motor.containsKey("pos") || !json_motor.containsKey("vel") || !json_motor.containsKey("cur")){
        //Serial.print("Error en los datos encontrados");
        clearI2C();
        return;
      }
      
      JsonDocument json_parametros;
      // Procesamiento para parámetros de posición
      json_parametros = json_motor["pos"];
      if (!json_parametros.containsKey("kc") || !json_parametros.containsKey("ti")){
        //Serial.println("Parámetros no encontrados");
        clearI2C();
        return;
      }
      posKP = json_parametros["kc"].as<int>();
      posKI = json_parametros["ti"].as<int>();
      json_parametros.clear();

      // Procesamiento para parámetros de velocidad
      json_parametros = json_motor["vel"];
      if (!json_parametros.containsKey("kc") || !json_parametros.containsKey("ti")){
        //Serial.println("Parámetros no encontrados");
        clearI2C();
        return;
      }
      velKP = json_parametros["kc"].as<int>();
      velKI = json_parametros["ti"].as<int>();
      json_parametros.clear();
      
      // Procesamiento para parámetros de corriente
      json_parametros = json_motor["cur"];
      if (!json_parametros.containsKey("kc") || !json_parametros.containsKey("ti")){
        //Serial.println("Parámetros no encontrados");
        clearI2C();
        return;
      }
      curKP = json_parametros["kc"].as<int>();
      curKI = json_parametros["ti"].as<int>();
      json_parametros.clear();

      // Se mandan los parámetros
      /*Serial.print("kc: "); 
      Serial.println(posKP);
      Serial.print("ti: ");
      Serial.println(posKI);
      Serial.print("kc: "); 
      Serial.println(velKP);
      Serial.print("ti: ");
      Serial.println(velKI);
      Serial.print("kc: "); 
      Serial.println(curKP);
      Serial.print("ti: ");
      Serial.println(curKI);*/
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
        //Serial.print("Información no encontrada");
        clearI2C();
        return;
      }

      // Se extraen los Set Points del mensaje
      SP_motor1 = jsonrec["motor1"].as<int>();
      SP_motor2 = jsonrec["motor2"].as<int>();
      SP_motor3 = jsonrec["motor3"].as<int>();
      
      // Control dependiendo del tipo 
      process_variable = jsonrec["monitoring"].as<String>();
      if (process_variable == "pos"){
        // Control de posición
        //Serial.println("Control de posición");
        set_absolute_position(1, SP_motor1, max_speed, true);
        delayMS(CAN_DELAY); // delay 
        set_absolute_position(2, SP_motor2, max_speed, true);
        delayMS(CAN_DELAY); // delay 
        //set_absolute_position(3, SP_motor3, max_speed, true);
        //delay(CAN_DELAY); // delay 
      }
      else if(process_variable == "vel"){
        // Control de velocidad
        //Serial.println("Control de velocidad");
        set_speed(1, SP_motor1, true);
        delayMS(CAN_DELAY); // delay 
        set_speed(2, SP_motor2, true);
        delayMS(CAN_DELAY); // delay 
        //set_speed(3, SP_motor3, true);
        //delayMS(CAN_DELAY); // delay 
      }
      else if (process_variable == "cur"){
        // Control de torque
        //Serial.println("Control de torque");
        set_torque(1, SP_motor1, false);
        delayMS(CAN_DELAY); // delay 
        set_torque(2, SP_motor2, false);
        delayMS(CAN_DELAY); // delay 
        //set_torque(3, SP_motor3, false);
        //delayMS(CAN_DELAY); // delay  
      }
    }
    else if (strcmp(type, "H") == 0){
      // ------- Comandos a los motores -------
      /* Ejemplo
        {"state": "stop", "T", "H"}
      */
      if (!jsonrec.containsKey("state")){
        //Serial.println("Información no disponible");
        return;  
      }
      const char* state_command = jsonrec["state"];
      if (strcmp(state_command, "stop") == 0){ // Comando de detenerse
        //Serial.println("Deteniendo motores");
        stop_motor(1, false);
        delayMS(CAN_DELAY); // delay 
        stop_motor(2, false);
        delayMS(CAN_DELAY); // delay 
        stop_motor(3, false);
        delayMS(CAN_DELAY); // delay 
      }
    }
  
    // Se limpia el buffer y el archivo json receptor
    mensaje_leido = "";
  }
}

void onRequest(){
  // Función de callback que se ejecuta al recibir un request por I2C

  // Se manda un fragmento del mensaje
  int bytesToSend = min(32, stringsend_ESP32.length() - index_alt);
  Wire.write(stringsend_ESP32.substring(index_alt, index_alt + bytesToSend).c_str(), bytesToSend);
  //Serial.println(stringsend_ESP32.substring(index_alt, index_alt + bytesToSend).c_str());
  index_alt += bytesToSend;

  if (index_alt >= stringsend_ESP32.length()) { // Se llega al final del mensaje
    index_alt = 0;       
  }
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
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    SysTickIntRegister(ISRSysTick);
    SysTickPeriodSet(16777215);
    SysTickIntEnable();
    SysTickEnable();

    // ------ Configuración de I2C ------
    Wire.begin(I2C_DEV_ADDR);               // Inicializa el protocolo I2C
    Wire.onReceive(onReceive); // Se registra el evento de onreceive
    Wire.onRequest(onRequest); // register event

    IntMasterEnable();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0)) {}
   
    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 1000000u);
    CANEnable(CAN0_BASE);
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0); // test
    CANIntRegister(CAN0_BASE,CAN0IntHandler);
    Serial.println("Listo");
}

// ----- Main Loop -----
void loop() {
 //if (doControlFlag) {
    //doControlFlag = false;
    //GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, RED_LED); 

  if (process_variable == "pos"){
    delayMS(20);
    read_angle(1);
    delayMS(20);
    read_angle(2);
  } else if (process_variable == "vel"){
    delayMS(20);
    read_velocity(1);
    delayMS(20);
    read_velocity(2); 
  } else if (process_variable == "cur"){
    delayMS(20);
    read_current(1);
    delayMS(20);
    read_current(2); 
  }
   //GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, LOW); 
 //}
  //Serial.print(process_variable);
  //Serial.print(": ");
  //Serial.print(PV1);
  //Serial.print(", ");
  //Serial.println(PV2);
}
