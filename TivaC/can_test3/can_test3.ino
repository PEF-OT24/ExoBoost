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

// Conversión de unidades
#define DEG_TO_RAD (3.141592653589793 / 180.0)  // Conversión de grados a radianes
#define RAD_TO_DEG (180.0 / 3.141592653589793)  // Conversión de radianes a grados

// Se define CAN_INT_INTID_STATUS si no lo está
#ifndef CAN_INT_INTID_STATUS
#define CAN_INT_INTID_STATUS 0x8000
#endif

// Delay entre mensajes de CAN en ms
#define CAN_DELAY 100
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
uint16_t max_speed = 500; // Leer Nota 
/*
Nota: max_spsed controla la velocidad máxima con la que se ejecutan comandos de control de posición. 
Si max_speed = 0, entonces la velocidad será la calculada por el controlador PI, de otra manera, 
se limita al valor indicado. 
*/

// Variables de proceso para los motores
String process_variable = "pos"; // Tipo de variable de proceso: pos, vel, cur, temp
int32_t PV1 = 1;
int32_t PV2 = 1;
int32_t PV3 = 1;

// Variables de lectura de corriente en formato float
int16_t PV1_cur;
int16_t PV2_cur;
int16_t PV3_cur;
uint8_t current_Array[2];
bool walk_flag;

bool doControlFlag = 0; // Bandera de control en tiempo real 
int8_t current_tab = 0; // Variable que almacena la ventana en la que se encuentra el usuario
int8_t last_tab = 0;       // Variable que almacena la ventana antigua en la que se encontraba el usuario
bool resetFlag = false; // Bandera que indica si es necesario un reset
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
uint8_t motor_selected = 0;   // Indicador del motor seleccionado para lectura

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

void print_data(uint8_t arr[8]) {
  // Inicia la comunicación serial
  Serial.begin(9600);

  // Recorre el array e imprime cada valor en formato hexadecimal
  for (int i = 0; i < 8; i++) {
    if (arr[i] < 0x10) {
      // Si el valor es menor que 0x10, imprime un 0 para mantener formato de 2 dígitos
      //Serial.print("0");
    }
    //Serial.print(arr[i], HEX);
    
    // Imprime un espacio entre los valores, pero no después del último
    if (i < 7) {
      //Serial.print(" ");
    }
  }
  
  // Salto de línea al final
  //Serial.println();
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
            int16_t PV1_read = ((CANBUSReceive[3] << 8) | CANBUSReceive[2]);
            PV1_cur = PV1_read;
            // Serial.print("PV1 "); Serial.print(PV1_read); Serial.print(" ");
            //print_data(CANBUSReceive); 
          }
          Serial.print("PV1: "); Serial.println(PV1);

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
            int16_t PV2_read = ((CANBUSReceive[3] << 8) | CANBUSReceive[2]);
            PV2_cur = PV2_read; // Lectura en cA
            // Serial.print("PV2 "); Serial.println(PV2_read);
            //print_data(CANBUSReceive);
          }
          Serial.print("PV2: "); Serial.println(PV2);
          
        } else if (motor_selected == 3){           // motor 3
          // Se obtiene el mensaje
          Message_Rx_3.pui8MsgData = CANBUSReceive;
          CANMessageGet(CAN0_BASE, 3, &Message_Rx_3, false);
  
          // Validación del mensaje
          if (commandCAN != CANBUSReceive[0]){
            return;
          }
          // Serial.print("M");Serial.print(ui32Status);Serial.print(" ");
          
          // Se formatea la información dependiendo del comando recibido
          if (process_variable == "pos"){ // Formateo para posición
            PV3 = int(round(((CANBUSReceive[7] << 24) | (CANBUSReceive[6] << 16) | (CANBUSReceive[5] << 8) | CANBUSReceive[4])/100));  
          } else if (process_variable == "vel"){ // Formateo para velocidad
            int16_t PV3_read;
            PV3_read = int(round((CANBUSReceive[5] << 8) | CANBUSReceive[4]));  
            PV3 = PV3_read;
          } else if (process_variable == "cur"){ // Formateo para corriente
            int16_t PV3_read = ((CANBUSReceive[3] << 8) | CANBUSReceive[2]);
            PV3_cur = PV3_read; // Lectura en cA
            // Serial.print("PV3 "); Serial.println(PV3_cur);
            //print_data(CANBUSReceive);
          }
          Serial.print("PV3: "); Serial.println(PV3);
        }

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

void motion_send_cmd(uint8_t ID, uint8_t *messageArray, bool show){ // Función para enviar un mensaje por CAN
  // Objetos para la comunicación CAN
  tCANMsgObject Message_Tx;
  
  uint8_t CAN_data_RX[8u];

  // Define el mensaje para mandar por CAN
  Message_Tx.ui32MsgID = 0x400 + ID;
  Message_Tx.ui32MsgIDMask = 0xFFFFFFFF;
  Message_Tx.ui32MsgLen = 8u;
  // Message_Tx.ui32Flags = MSG_OBJ_TX_INT_ENABLE; // Habilita interrupciones en el envío de mensaje
  Message_Tx.pui8MsgData = messageArray;

  if (ID == 1){       // Establecimiento de mensajes para el motor 1
    // Define el mensaje para leer por CAN
    Message_Rx_1.ui32MsgID = 0x501;
    Message_Rx_1.ui32MsgIDMask = 0xFFFFFFFF; // Lee todos los mensajes
    Message_Rx_1.ui32Flags = MSG_OBJ_RX_INT_ENABLE; // Habilita interrupciones en la recepción del mensaje
    Message_Rx_1.ui32MsgLen = 8u;
    Message_Rx_1.pui8MsgData = CAN_data_RX;
  } else if (ID == 2){ // Establecimiento de mensajes para el motor 2
    // Define el mensaje para leer por CAN
    Message_Rx_2.ui32MsgID = 0x502;
    Message_Rx_2.ui32MsgIDMask = 0xFFFFFFFF; 
    Message_Rx_2.ui32Flags = MSG_OBJ_RX_INT_ENABLE; 
    Message_Rx_2.ui32MsgLen = 8u;
    Message_Rx_2.pui8MsgData = CAN_data_RX;
  } else if (ID == 3){ // Establecimiento de mensajes para el motor 3
    // Define el mensaje para leer por CAN
    Message_Rx_3.ui32MsgID = 0x503;
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

void stop_all_motors(){
  // Función para detener el motor
  
  stop_motor(1, false);
  delayMS(CAN_DELAY);
  stop_motor(2, false);
  delayMS(CAN_DELAY);
  stop_motor(3, false);
  delayMS(CAN_DELAY);
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

void reset_all_motors(){
  // Función para mandar un stop y shutdown a los motores con ID 1, 2 y 3
  if (last_tab == 2 || current_tab == 2 || current_tab == 1){
    stop_all_motors();
  
    delayMS(100); // delay de 100 ms entre el stop y shutdown
    shutdown_motor(1, false);
    delayMS(CAN_DELAY);   
    shutdown_motor(2, false);
    delayMS(CAN_DELAY);
    shutdown_motor(3, false);
  
  }
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

void motion_mode_command(int8_t ID, float p_des_deg, float v_des_deg_per_s, float kp, float kd, float t_ff, bool show) {
    uint16_t p_des_hex, v_des_hex, kp_hex, kd_hex, t_ff_hex;

    // Convert position from degrees to radians and normalize
    float p_des_rad = p_des_deg * DEG_TO_RAD;
    p_des_hex = (uint16_t)(((p_des_rad + 12.5) / 25.0) * 65535);

    // Convert velocity from degrees/second to radians/second and normalize
    float v_des_rad_per_s = v_des_deg_per_s * DEG_TO_RAD;
    v_des_hex = (uint16_t)(((v_des_rad_per_s + 45.0) / 90.0) * 4095);
    v_des_hex &= 0x0FFF; //Mask para asegurar que la variable entra en 12 bits

    // Normalize kp to the range 0 to 500
    kp_hex = (uint16_t)((kp / 500.0) * 4095);
    kp_hex &= 0x0FFF;

    // Normalize kd to the range 0 to 5
    kd_hex = (uint16_t)((kd / 5.0) * 4095);
    kd_hex &= 0x0FFF;

    // Normalize feedforward torque to the range -24 to 24 Nm
    t_ff_hex = (uint16_t)(((t_ff + 24.0) / 48.0) * 4095);
    t_ff_hex &= 0x0FFF; 

    // Pack the CAN data into an 8-byte array
    uint8_t CAN_message[8];

    // Packing into CAN_message array
    CAN_message[0] = (p_des_hex >> 8) & 0xFF;         // High byte of p_des
    CAN_message[1] = p_des_hex & 0xFF;                // Low byte of p_des
    CAN_message[2] = ((v_des_hex >> 4) & 0xFF);       // High 8 bits of v_des
    CAN_message[3] = ((v_des_hex & 0x0F) << 4)        // Low 4 bits of v_des
                     | ((kp_hex >> 8) & 0x0F);        // High 4 bits of kp
    CAN_message[4] = kp_hex & 0xFF;                   // Low byte of kp
    CAN_message[5] = ((kd_hex >> 4) & 0xFF);          // High 8 bits of kd
    CAN_message[6] = ((kd_hex & 0x0F) << 4)           // Low 4 bits of kd
                     | ((t_ff_hex >> 8) & 0x0F);      // High 4 bits of t_ff
    CAN_message[7] = t_ff_hex & 0xFF;                 // Low byte of t_ff

    // Send the CAN message (replace send_CAN with your CAN send function)
    motion_send_cmd(ID, CAN_message , true);
}

void walk_mode_sequence(float kp, float kd){
//  int angle_sim_hip[8] = {30, 22, 3, 10, 3, 17, 27, 30};
//  int angle_sim_knee[8] = {3, 12, 3, 3, 35, 55, 25, 3};
  float angle_sim_ankle[100] = {0,5.708152,11.39332,17.03261,22.60332,28.08301,33.44962,38.68154,43.75771,48.65768,53.36171,57.85089,62.10711,66.11326,69.85318,73.31184,76.47529,79.3308,81.86687,84.0733,85.9412,87.46304,88.6327,89.44547,89.89806,89.98867,89.71693,89.08393,88.09222,86.7458,85.05007,83.01189,80.63944,77.94228,74.93128,71.61856,68.01746,64.14247,60.00921,55.63431,51.03539,46.23097,41.24039,36.08375,30.78181,25.35593,19.82795,14.22013,8.555044,2.855514,-2.855514,-8.555044,-14.22013,-19.82795,-25.35593,-30.78181,-36.08375,-41.24039,-46.23097,-51.03539,-55.63431,-60.00921,-64.14247,-68.01746,-71.61856,-74.93128,-77.94228,-80.63944,-83.01189,-85.05007,-86.7458,-88.09222,-89.08393,-89.71693,-89.98867,-89.89806,-89.44547,-88.6327,-87.46304,-85.9412,-84.0733,-81.86687,-79.3308,-76.47529,-73.31184,-69.85318,-66.11326,-62.10711,-57.85089,-53.36171,-48.65768,-43.75771,-38.68154,-33.44962,-28.08301,-22.60332,-17.03261,-11.39332,-5.708152,-2.204364e-14};

  
  for (int i = 0; i<=100; i++){
//    read_angle(1);
//    delayMS(CAN_DELAY);
//    read_angle(2);
// //    delayMS(CAN_DELAY);
//     read_angle(3);
//     delayMS(CAN_DELAY);
//    motion_mode_command(1,angle_sim_hip[i],0,1.4,0.1,0,true);
//    delayMS(CAN_DELAY); // delay 
//    motion_mode_command(2,angle_sim_knee[i],0,1.1,0.05,0,true);
//    delayMS(CAN_DELAY);
    motion_mode_command(3,angle_sim_ankle[i],0,1.1,0.05,0,true);
    delayMS(CAN_DELAY);
//    read_angle(1);
//    delayMS(CAN_DELAY);
//    read_angle(2);
//    delayMS(CAN_DELAY);
    // read_angle(3);
    
    // delayMS(100); // delay
    
  }
}

// ----------------------------------- Funciones de callback de manejo de I2C -----------------------------------
// Función para reiniciar el bus I2C ante algún error
void clearI2C() {
  while (Wire.available()) {
    Wire.read();  // Lee y desecha cualquier dato restante en el buffer
  }
  mensaje_leido = ""; // Reinicia el buffer manual
}

void onReceive(int len){
  // Función de callback que se ejecuta al recibir un mensaje por I2C

  // Lee los bytes recibidos
  while (Wire.available()) { // Guarda los bytes recibidos
    char rec_data = Wire.read();
    mensaje_leido += rec_data;
  }

  if (mensaje_leido.endsWith("\n")){ // Indicador de que el mensaje está completo
    mensaje_leido.trim();            // Se elimina el terminador 
    JsonDocument jsonrec;            // Archivo json para recibir información 
  
    // ----- Procesamiento del mensaje recibido -----
    const char *value = mensaje_leido.c_str();
    DeserializationError error = deserializeJson(jsonrec, value);
  
    // Se revisan errores
    if (error) { // Error al deserializar
      // Serial.print("Error al analizar JSON: ");
      // Serial.println(error.c_str());
      clearI2C();
      return;
    }
    else if (!jsonrec.containsKey("T")){ // Si no contiene el tipo
      //Serial.println("Tipo no encontrado en el JSON");
      clearI2C();
      return;
    }
    else if (!jsonrec["T"].is<const char*>()){ // Contenido en "tipo" incorrecto
      //Serial.println("Tipo de dato erróneo");
      clearI2C();
      return;   
    }
  
    // ----------- Procesamiento del JSON completo --------
    // El objeto "T" contiene un indicador de qué tipo de información contiene el JSON
    const char* type = jsonrec["T"];
    // Serial.println(type);
    if (strcmp(type, "A") == 0){ 
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
      // Serial.print("Nivel de asistencia: ");
      // Serial.println(assistance_level);
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
      if (resetFlag){ // Se resetean los motores si es necesario
        reset_all_motors();
        resetFlag = false;
      }
      
      if (process_variable == "pos"){
        // Control de posición
        Serial.println("Control de posición");
        set_absolute_position(1, SP_motor1, max_speed, false);
        delayMS(CAN_DELAY); // delay 
        set_absolute_position(2, SP_motor2, max_speed, false);
        delayMS(CAN_DELAY); // delay 
        set_absolute_position(3, SP_motor3, max_speed, false);
        delayMS(CAN_DELAY); // delay 
      }
      else if(process_variable == "vel"){
        // Control de velocidad
        //Serial.println("Control de velocidad");
        set_speed(1, SP_motor1, false);
        delayMS(CAN_DELAY); // delay 
        set_speed(2, SP_motor2, false);
        delayMS(CAN_DELAY); // delay 
        set_speed(3, SP_motor3, false);
        delayMS(CAN_DELAY); // delay 
      }
      else if (process_variable == "cur"){
        // Control de torque
        //Serial.println("Control de torque");
        set_torque(1, SP_motor1, false);
        delayMS(CAN_DELAY); // delay 
        set_torque(2, SP_motor2, false);
        delayMS(CAN_DELAY); // delay 
        set_torque(3, SP_motor3, false);
        delayMS(CAN_DELAY); // delay  
      }
    }
    else if (strcmp(type, "F") == 0){ 
      // --------------- Formateo Variable de Proceso ---------
      // Sección que establece qué variable de proceso se está midiendo
      // Guarda la información sobre dicha variable en un archivo JSON 
      
      jsonsend_ESP32.clear();    // Se limpia el JSON anterior
      stringsend_ESP32 = "";     // Se reinicia el string anterior

      // Se define el mensaje con los valores de la variable de proceso
      jsonsend_ESP32["monitoring"] = process_variable;
      jsonsend_ESP32["limb"] = "Right leg";
      jsonsend_ESP32["motor1"] = String(PV1);
      jsonsend_ESP32["motor2"] = String(PV2);
      jsonsend_ESP32["motor3"] = String(PV3);

      // Se construye el mensaje serializado para mandar
      serializeJson(jsonsend_ESP32, stringsend_ESP32);
      stringsend_ESP32 += '\n';  // terminador '\n'
    }
    else if (strcmp(type, "G") == 0){
      // ------------ Comando según ventana -----------
      /*
      La lógica interna de la Tiva C cambia en función de en qué ventana se 
      encuentre el usuario. 
      Ventanas: 
      1 - Bluetooth - Ninguna acción
      2 - Assistance - Motion Mode activado, requiere reset
      3 - Tuning - Servo Mode activado, requiere reset. 
      4 - Monitoring - Lectura de PV activada

      Nota: Si una acción no indica activación, está desactivada. 
      */
      if (!jsonrec.containsKey("tab")){ // Validación del mensaje
        return;  
      }

      last_tab = current_tab; // Se guarda la tab anterior para reset motores
      current_tab = jsonrec["tab"].as<int8_t>(); // Se extrae la información
      // Serial.print("Tab: ");
      // Serial.println(current_tab);
      if (current_tab < 0 || current_tab > 4){
        return; // Error encontrado en el valor  
      }

      // Si hay un cambio de tab, se resetean los motores
      if (current_tab != last_tab){
        reset_all_motors();
        resetFlag = true;
      }
    }
    else if (strcmp(type, "H") == 0){
      // ------- Comandos a los motores -------
      /* Ejemplo
        {"state": "stop", "T", "H"}
      */
      if (!jsonrec.containsKey("state")){ // Validación del mensaje
        //Serial.println("Información no disponible");
        return;  
      }
      
      const char* state_command = jsonrec["state"];
      if (strcmp(state_command, "stop") == 0){ // Comando de detenerse
        //Serial.println("Deteniendo motores");
        walk_flag = 0;
        stop_motor(1, false);
        delayMS(CAN_DELAY); // delay 
        stop_motor(2, false);
        delayMS(CAN_DELAY); // delay 
        stop_motor(3, false);
        delayMS(CAN_DELAY); // delay
      }
       else if (strcmp(state_command, "walk") == 0){ // Comando de caminar
        walk_flag = 1;
      }
       else if (strcmp(state_command, "stand up") == 0){ // Comando de levantarse
        walk_flag = 0;
        /*
        read_angle(1);
        delayMS(CAN_DELAY);
        read_angle(2);
        delayMS(CAN_DELAY);
        read_angle(3);
        delayMS(CAN_DELAY);
        */
        motion_mode_command(1,+90,0,1,0.1,0,false); 
        delayMS(CAN_DELAY); // delay 
        motion_mode_command(2,-90,0,1,0.1,0,false);
        delayMS(CAN_DELAY);
        motion_mode_command(3,0,0,1,0.1,0,false);
        delayMS(CAN_DELAY); // delay
        /*
        delayMS(CAN_DELAY);
        read_angle(1);
        delayMS(CAN_DELAY);
        read_angle(2);
        delayMS(CAN_DELAY);
        read_angle(3);
        */
      }
       else if (strcmp(state_command, "sit down") == 0){ // Comando de sentarse
        walk_flag = 0;
        /*
        read_angle(1);
        delayMS(CAN_DELAY);
        read_angle(2);
        delayMS(CAN_DELAY);
        read_angle(3);
        delayMS(CAN_DELAY);
        */
        motion_mode_command(1,-90,0,1,0.1,0,false);
        delayMS(CAN_DELAY); // delay 
        motion_mode_command(2,+90,0,1,0.1,0,false);
        delayMS(CAN_DELAY); // delay
        motion_mode_command(3,0,0,1,0.1,0,false);
        delayMS(CAN_DELAY); // delay
        /*
        delayMS(CAN_DELAY);
        read_angle(1);
        delayMS(CAN_DELAY);
        read_angle(2);
        delayMS(CAN_DELAY);
        read_angle(3);
        */
      }
    }
    // Se limpia el buffer y el archivo json receptor
    mensaje_leido = "";
  }
}

void onRequest(){
  // Función de callback que se ejecuta al recibir un request por I2C
  // Nota: El mensaje a mandar se debe definir antes de que se haga el request
  
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

    /*
    SysTickIntRegister(ISRSysTick);
    SysTickPeriodSet(11200);
    SysTickIntEnable();
    SysTickEnable();
    */

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

    // Set up de lectura de corriente
    process_variable = "cur";

}
// ----- Main Loop -----

void loop() {
  read_current(3);
  delayMS(10);
  split16bits(PV3_cur, current_Array);

  Serial.write(current_Array, 2);
}

// void loop() {
//   if (walk_flag == 1 && current_tab == 2){ // On assistance tab
//     walk_mode_sequence(1.4,0.05);
    
//     if (resetFlag){ // Se resetean los motores si es indicado y se baja la bandera
//       reset_all_motors();
//       resetFlag = false;
//     }
//   }
  
//   if (current_tab == 4) { // On monitoring tab
    
//     if (process_variable == "pos"){
//       delayMS(80);
//       read_angle(1);
//       delayMS(80);
//       read_angle(2);
//       delayMS(80);
//       read_angle(3);
//     } else if (process_variable == "vel"){
//       delayMS(80);
//       read_velocity(1); 
//       delayMS(80);
//       read_velocity(2); 
//       delayMS(80);
//       read_velocity(3); 
    
//     } else if (process_variable == "cur"){
//       delayMS(80);
//       read_current(1);
//       delayMS(80);
//       read_current(2); 
//       delayMS(80);
//       read_current(3); 
//     }
//   }
// }

void read_currents(){
  process_variable = "cur";
  delayMS(CAN_DELAY);
  read_current(1);
  delayMS(CAN_DELAY);
  read_current(2); 
  delayMS(CAN_DELAY);
  read_current(3);  
}

void read_positions(){
  delayMS(CAN_DELAY);
  read_angle(1);
  delayMS(CAN_DELAY);
  read_angle(2); 
  delayMS(CAN_DELAY);
  read_angle(3);  
}
