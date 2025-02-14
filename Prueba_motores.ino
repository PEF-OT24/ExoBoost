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
#include "driverlib/timer.h"
#include "driverlib/can.h"
#include "driverlib/systick.h"
#include <Wire.h>
#include <ArduinoJson.h>


// Definición de pines LED
#define RED_LED GPIO_PIN_1
#define BLUE_LED GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3
#define NOTIFY_PIN GPIO_PIN_4

// Conversión de unidades
#define DEG_TO_RAD (3.141592653589793 / 180.0)  // Conversión de grados a radianes
#define RAD_TO_DEG (180.0 / 3.141592653589793)  // Conversión de radianes a grados

// Se define CAN_INT_INTID_STATUS si no lo está
#ifndef CAN_INT_INTID_STATUS
#define CAN_INT_INTID_STATUS 0x8000
#endif

// Delay entre mensajes de CAN en ms
#define CAN_DELAY 15
#define STEP_DELAY 33
#define KNEE_DELAY 60

byte commandCAN = 0x00;   


// Sintonización
// Cadera
float kp_hip = 3;
float kd_hip = 0;
float tff_hip = 0;

// Rodilla
float kp_knee = 2.7;
float kd_knee = 0.1;
float tff_knee = 0;

// Tobillo
float kp_ankle = 8;
float kd_ankle = 0.5;
float tff_ankle = 0;

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
uint16_t max_speed = 200; // Leer nota

/*
  Nota: max_spsed controla la velocidad máxima con
   la que se ejecutan comandos de control de posición.
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
uint8_t current1_Array[2];
uint8_t current2_Array[2];
uint8_t current3_Array[2];

tCANMsgObject Message_Rx_1;   // Objeto para leer mensajes del motor 1
tCANMsgObject Message_Rx_2;   // Objeto para leer mensajes del motor 2
tCANMsgObject Message_Rx_3;   // Objeto para leer mensajes del motor 3
uint8_t motor_selected = 0;   // Indicador del motor seleccionado para lectura

//--------------------- Variables para el algoritmo de caminata ---------------------
bool walk_flag = false;  // Bandera de caminata
bool stop_flag = false;  // Bandera de stop
bool notified = false;   // Bandera de notificación en fase 0

int count = 0; // Conteo de setpoints
uint8_t gait_phase = 0; // Fase de caminata
/*
  gait_phase = 0: espera
  gait_phase = 1: Balanceo
  gait_phase = 2: Contacto Inicial
  gait_phase = 3: Apoyo
  gait_phase = 4: Pre Balanceo
*/

// -- Vectores de caminata --
// Para cada fase, el tamaño de los vectores por articulación debe ser el mismo

// Fase 1 - Balanceo (30 valores)
int16_t hip_balanceo[5] = {10, 30, 30, 30, 30};
int16_t knee_balanceo[5] = {45, 60, 40, 20, 0};
int16_t ankle_balanceo[5] = { -20, 10, 0, 0, 0};

// Fase 2 - Contacto Inicial (1 valores)
int16_t hip_contacto_inicial[5] = {30, 27, 25, 22, 20};
int16_t knee_contacto_inicial[5] = {4, 7, 10, 12, 15};
int16_t ankle_contacto_inicial[5] = {0, -2, -5, -7, -10};

// Fase 3 - Apoyo (1 valores)
int8_t hip_apoyo[4] = {20, 10, 0, -10};
int16_t knee_apoyo[4] = {15, 10, 5, 0};
int16_t ankle_apoyo[4] = { -10, 0, 10, 15};

// Fase 4 - Pre balanceo (2 valores)
int16_t hip_pre_balanceo[3] = { -10, 0, 10}; // -10 a 0
int16_t knee_pre_balanceo[3] = {5, 15, 30}; // 0 a 30
int16_t ankle_pre_balanceo[3] = {15, 0, -20}; // 15 a -20

void LED(const char* color) { // Enciende el LED RGB integrado según el comando indicado
    if (strcmp(color, "GREEN") == 0) {
      GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, GREEN_LED); // Verde
    }
    else if (strcmp(color, "RED") == 0) {
      GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, RED_LED);   // Rojo
    }
    else if (strcmp(color, "BLUE") == 0) {
      GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, BLUE_LED);  // Azul
    }
    else if (strcmp(color, "OFF") == 0) {
      GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, 0); // Apagado
    }
    else if (strcmp(color, "YELLOW") == 0) {
      GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, RED_LED | GREEN_LED); // Amarillo
    }
    else if (strcmp(color, "PURPLE") == 0) {
      GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, RED_LED | BLUE_LED); // Morado
    }
    else if (strcmp(color, "CYAN") == 0) {
      GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, GREEN_LED | BLUE_LED); // Morado
    }
    else if (strcmp(color, "WHITE") == 0) {
      GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, RED_LED | GREEN_LED | BLUE_LED); // Blanco
    }
  }

void ConfigCAN() {
    // Habilitar pines del puerto B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {} // En espera de habilitar
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5); // CAN en PB4 y PB5
    IntMasterEnable();
  
    // Habilitar módulo CAN0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0)) {} // En espera de habilitar
    CANInit(CAN0_BASE);                                   // Inicializar módulo CAN0
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 1000000u); // Establecer la velocidad de transmisión
    CANEnable(CAN0_BASE);                                 // Habilitar módulo CAN0
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0);                                  // Habilitar interrupción
    CANIntRegister(CAN0_BASE, CAN0IntHandler);            // Registrar interrupción
}

void CAN0IntHandler(void) { // Función de interrupción para recepción de mensajes de CAN
    uint8_t CANBUSReceive[8u];
    uint32_t ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
  
    // Revisión de interrupción por un cambio de estado
    if (ui32Status == CAN_INT_INTID_STATUS) {
      // Leer el estado de módulo CAN0
      ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
  
    }
  
    // Interrupción por motores 1, 2 ó 3
    else if (ui32Status == 1 || ui32Status == 2 || ui32Status == 3) {
      // Limpia el interrupt
      CANIntClear(CAN0_BASE, ui32Status);
  
      // No realiza lectura si no es necesario
      if (commandCAN != 0x92 && commandCAN != 0x9C) {
        Serial.println("NO MSG");
        return; // Solo lectura de posición, velocidad o corriente
      }
  
      // -------- Lectura del motor 1 ---------
      if (motor_selected == 1) {
        // Se obtiene el mensaje
        Message_Rx_1.pui8MsgData = CANBUSReceive;
        CANMessageGet(CAN0_BASE, 1, &Message_Rx_1, false);
  
        // Validación del mensaje
        if (commandCAN != CANBUSReceive[0]) {
          return;
        }
  
        // Formato de recepción
        if (process_variable == "pos") {       // Formateo para posición
          PV1 = int(round(((CANBUSReceive[7] << 24) | (CANBUSReceive[6] << 16) | (CANBUSReceive[5] << 8) | CANBUSReceive[4]) / 100));
        } else if (process_variable == "vel") { // Formateo para velocidad
          int16_t PV1_read;
          PV1_read = int(round((CANBUSReceive[5] << 8) | CANBUSReceive[4]));
          PV1 = PV1_read;
        } else if (process_variable == "cur") { // Formateo para corriente
          uint16_t PV1_read = ((CANBUSReceive[3] << 8) | CANBUSReceive[2]);
          PV1_cur = PV1_read;
        }
  
        // -------- Lectura del motor 2 ---------
      } else if (motor_selected == 2) {
        // Se obtiene el mensaje
        Message_Rx_2.pui8MsgData = CANBUSReceive;
        CANMessageGet(CAN0_BASE, 2, &Message_Rx_2, false);
  
        // Validación del mensaje
        if (commandCAN != CANBUSReceive[0]) {
          return;
        }
  
        // Se formatea la información dependiendo del comando recibido
        if (process_variable == "pos") {        // Formateo para posición
          PV2 = int(round(((CANBUSReceive[7] << 24) | (CANBUSReceive[6] << 16) | (CANBUSReceive[5] << 8) | CANBUSReceive[4]) / 100));
        } else if (process_variable == "vel") { // Formateo para velocidad
          int16_t PV2_read;
          PV2_read = int(round((CANBUSReceive[5] << 8) | CANBUSReceive[4]));
          PV2 = PV2_read;
        } else if (process_variable == "cur") { // Formateo para corriente
          uint16_t PV2_read = ((CANBUSReceive[3] << 8) | CANBUSReceive[2]);
          PV2_cur = PV2_read; // Lectura en cA
        }
  
        // -------- Lectura del motor 3 ---------
      } else if (motor_selected == 3) {
        // Se obtiene el mensaje
        Message_Rx_3.pui8MsgData = CANBUSReceive;
        CANMessageGet(CAN0_BASE, 3, &Message_Rx_3, false);
  
        // Validación del mensaje
        if (commandCAN != CANBUSReceive[0]) {
          return;
        }
  
        // Se formatea la información dependiendo del comando recibido
        if (process_variable == "pos") {        // Formateo para posición
          PV3 = int(round(((CANBUSReceive[7] << 24) | (CANBUSReceive[6] << 16) | (CANBUSReceive[5] << 8) | CANBUSReceive[4]) / 100));
        } else if (process_variable == "vel") { // Formateo para velocidad
          int16_t PV3_read;
          PV3_read = int(round((CANBUSReceive[5] << 8) | CANBUSReceive[4]));
          PV3 = PV3_read;
        } else if (process_variable == "cur") { // Formateo para corriente
          int16_t PV3_read = ((CANBUSReceive[3] << 8) | CANBUSReceive[2]);
          PV3_cur = PV3_read; // Lectura en A / 100
        }
      }
  
    } else {
      CANIntClear(CAN0_BASE, ui32Status); // Eliminar interrupciones inesperadas
    }
  }

void setup() {
    // Habilitar interfaz serial
    Serial.begin(9600);
  
    // ----------- Habilitar periféricos -----------
    // Habilitar puerto F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED);
  
    // Habilita el puerto D
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {}
    // Configura PD_0 y PD_1 como entradas digitales sin pull-up o pull-down
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
  
    // Habilitar frecuencia a 80 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
  
    // ------ Configuración de ADC ------
    //ConfigADC();
  
    // ------ Configuración de UART ------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
  
    // ------ Configuración de CAN ------
    ConfigCAN();  
    
    // Set up de lectura de corriente
    process_variable = "cur";
  
    Serial.println("Listo!");
  }

  void motion_mode_command(int8_t ID, float p_des_deg, float v_des_deg_per_s, float kp, float kd, float t_ff) {

    uint16_t p_des_hex, v_des_hex, kp_hex, kd_hex, t_ff_hex;
  
    // Convertir posición de grados a radianes y normalizar
    float p_des_rad = p_des_deg * DEG_TO_RAD;
    p_des_hex = (uint16_t)(((p_des_rad + 12.5) / 25.0) * 65535);
  
    // Convertir velocidad de grados por segundo a radianes por segundo y normalizar
    float v_des_rad_per_s = v_des_deg_per_s * DEG_TO_RAD;
    v_des_hex = (uint16_t)(((v_des_rad_per_s + 45.0) / 90.0) * 4095);
    v_des_hex &= 0x0FFF; // Máscara para asegurar que la variable entra en 12 bits
  
    // Normalizar kd de 0 a 500
    kp_hex = (uint16_t)((kp / 500.0) * 4095);
    kp_hex &= 0x0FFF;    // Máscara de 12 bits
  
    // Normalizar kd de 0 a 5
    kd_hex = (uint16_t)((kd / 5.0) * 4095);
    kd_hex &= 0x0FFF;
  
    // Normalizar torque de feedforward de -24 a 24
    t_ff_hex = (uint16_t)(((t_ff + 24.0) / 48.0) * 4095);
    t_ff_hex &= 0x0FFF;  // Máscara de 12 bits
  
    // Buffer de mensaje de CAN
    uint8_t CAN_message[8];
  
    // Llenar el Buffer del mensaje de CAN
    CAN_message[0] = (p_des_hex >> 8) & 0xFF;         // High byte de p_des
    CAN_message[1] = p_des_hex & 0xFF;                // Low byte de p_des
    CAN_message[2] = ((v_des_hex >> 4) & 0xFF);       // High 8 bits de v_des
    CAN_message[3] = ((v_des_hex & 0x0F) << 4)        // Low 4 bits de v_des
                     | ((kp_hex >> 8) & 0x0F);        // High 4 bits de kp
    CAN_message[4] = kp_hex & 0xFF;                   // Low byte de kp
    CAN_message[5] = ((kd_hex >> 4) & 0xFF);          // High 8 bits de kd
    CAN_message[6] = ((kd_hex & 0x0F) << 4)           // Low 4 bits de kd
                     | ((t_ff_hex >> 8) & 0x0F);      // High 4 bits de t_ff
    CAN_message[7] = t_ff_hex & 0xFF;                 // Low byte de t_ff
  
    // Se manda el mensaje por CAN
    motion_send_cmd(ID, CAN_message);
  }

  void delayMS(uint32_t milliseconds) {
    // Función que ejecuta un delay de manera aproximada (las interrupciones tienen prioridad)
    uint32_t delay_cycles;
  
    // Get the system clock frequency in Hz
    uint32_t sysClock = SysCtlClockGet();
  
    // Calculate the number of delay cycles needed for the given milliseconds
    // Avoid overflow by dividing first
    delay_cycles = (sysClock / 3000) * milliseconds;
  
    // Call SysCtlDelay to create the delay
    SysCtlDelay(delay_cycles);
  }

  void stop_motor(int8_t ID) {
    // Función para detener el motor
  
    // Objetos para la comunicación CAN
    uint8_t CAN_data_TX[8u];
    commandCAN = 0x81; // comando de lectura
  
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
    send_cmd(ID, CAN_data_TX);
  }

  void stop_all_motors() {
    // Función para detener el motor
    Serial.println("Stop all motors");
  
    stop_motor(1);
    delayMS(CAN_DELAY);
    stop_motor(2);
    delayMS(CAN_DELAY);
    stop_motor(3);
    delayMS(CAN_DELAY);
  }

  void send_cmd(uint8_t ID, uint8_t *messageArray) {
    // Función para enviar un mensaje por CAN
  
    // Objeto para la comunicación CAN
    tCANMsgObject Message_Tx;
  
    // Buffer para la recepción del mensaje
    uint8_t CAN_data_RX[8u];
  
    // Define el mensaje para mandar por CAN
    Message_Tx.ui32MsgID = 0x140 + ID;       // Se le asigna el ID de destino
    Message_Tx.ui32MsgIDMask = 0xFFFFFFFF;   // Se configura una máscara sin filtro de mensajes
    Message_Tx.ui32MsgLen = 8u;              // Longitud de 8 bytes
    Message_Tx.pui8MsgData = messageArray;   // Payload del mensaje
  
    if (ID == 1) {      // Lectura de mensajes para el motor 1
      Message_Rx_1.ui32MsgID = 0x241;
      Message_Rx_1.ui32MsgIDMask = 0xFFFFFFFF;
      Message_Rx_1.ui32Flags = MSG_OBJ_RX_INT_ENABLE; // Interrupción en recepción
      Message_Rx_1.ui32MsgLen = 8u;
      Message_Rx_1.pui8MsgData = CAN_data_RX;
    } else if (ID == 2) { // Lectura de mensajes para el motor 2
      Message_Rx_2.ui32MsgID = 0x242;
      Message_Rx_2.ui32MsgIDMask = 0xFFFFFFFF;
      Message_Rx_2.ui32Flags = MSG_OBJ_RX_INT_ENABLE; // Interrupción en recepción
      Message_Rx_2.ui32MsgLen = 8u;
      Message_Rx_2.pui8MsgData = CAN_data_RX;
    } else if (ID == 3) { // Lectura de mensajes para el motor 3
      Message_Rx_3.ui32MsgID = 0x243;
      Message_Rx_3.ui32MsgIDMask = 0xFFFFFFFF;
      Message_Rx_3.ui32Flags = MSG_OBJ_RX_INT_ENABLE; // Interrupción en recepción
      Message_Rx_3.ui32MsgLen = 8u;
      Message_Rx_3.pui8MsgData = CAN_data_RX;
    }
  
    // Envío por CAN
    CANMessageSet(CAN0_BASE, ID, &Message_Tx, MSG_OBJ_TYPE_TX);
  
    // Configura los objetos de mensajes
    if (ID == 1) {
      CANMessageSet(CAN0_BASE, 1, &Message_Rx_1, MSG_OBJ_TYPE_RX);
    } else if (ID == 2) {
      CANMessageSet(CAN0_BASE, 2, &Message_Rx_2, MSG_OBJ_TYPE_RX);
    } else if (ID == 3) {
      CANMessageSet(CAN0_BASE, 3, &Message_Rx_3, MSG_OBJ_TYPE_RX);
    }
  }
  
  void shutdown_motor(int8_t ID) {
    // Función para apagar el motor
  
    // Objetos para la comunicación CAN
    uint8_t CAN_data_TX[8u];
    commandCAN = 0x80; // comando de lectura
  
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
    send_cmd(ID, CAN_data_TX);
  }

  void reset_motor(int8_t ID) {
    // Función para restablecer los comandos enviados al motor
  
    // Objetos para la comunicación CAN
    uint8_t CAN_data_TX[8u];
    commandCAN = 0x76; // comando de lectura
  
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
    send_cmd(ID, CAN_data_TX);
  }
  
  void reset_all_motors() {
    Serial.println("Reset all motors");
    // Función para mandar un stop y shutdown a los motores en la red
    stop_all_motors();
    delayMS(CAN_DELAY);
    shutdown_motor(1);
    delayMS(CAN_DELAY);
    shutdown_motor(2);
    delayMS(CAN_DELAY);
    shutdown_motor(3);
  }
  
  void motion_send_cmd(uint8_t ID, uint8_t *messageArray) { // Función para enviar un mensaje por CAN
    // Objetos para la comunicación CAN
    tCANMsgObject Message_Tx;
  
    uint8_t CAN_data_RX[8u];
  
    // Define el mensaje para mandar por CAN
    Message_Tx.ui32MsgID = 0x400 + ID;
    Message_Tx.ui32MsgIDMask = 0xFFFFFFFF;
    Message_Tx.ui32MsgLen = 8u;
    // Message_Tx.ui32Flags = MSG_OBJ_TX_INT_ENABLE; // Habilita interrupciones en el envío de mensaje
    Message_Tx.pui8MsgData = messageArray;
  
    if (ID == 1) {      // Establecimiento de mensajes para el motor 1
      // Define el mensaje para leer por CAN
      Message_Rx_1.ui32MsgID = 0x501;
      Message_Rx_1.ui32MsgIDMask = 0xFFFFFFFF; // Lee todos los mensajes
      //Message_Rx_1.ui32Flags = MSG_OBJ_RX_INT_ENABLE; // Habilita interrupciones en la recepción del mensaje
      Message_Rx_1.ui32MsgLen = 8u;
      Message_Rx_1.pui8MsgData = CAN_data_RX;
    } else if (ID == 2) { // Establecimiento de mensajes para el motor 2
      // Define el mensaje para leer por CAN
      Message_Rx_2.ui32MsgID = 0x502;
      Message_Rx_2.ui32MsgIDMask = 0xFFFFFFFF;
      //Message_Rx_2.ui32Flags = MSG_OBJ_RX_INT_ENABLE;
      Message_Rx_2.ui32MsgLen = 8u;
      Message_Rx_2.pui8MsgData = CAN_data_RX;
    } else if (ID == 3) { // Establecimiento de mensajes para el motor 3
      // Define el mensaje para leer por CAN
      Message_Rx_3.ui32MsgID = 0x503;
      Message_Rx_3.ui32MsgIDMask = 0xFFFFFFFF;
      //Message_Rx_3.ui32Flags = MSG_OBJ_RX_INT_ENABLE;
      Message_Rx_3.ui32MsgLen = 8u;
      Message_Rx_3.pui8MsgData = CAN_data_RX;
    }
      // Envío por CAN
    CANMessageSet(CAN0_BASE, ID, &Message_Tx, MSG_OBJ_TYPE_TX);

  // Configura los objetos de mensajes
    if (ID == 1) {
      CANMessageSet(CAN0_BASE, 1, &Message_Rx_1, MSG_OBJ_TYPE_RX);
    } else if (ID == 2) {
      CANMessageSet(CAN0_BASE, 2, &Message_Rx_2, MSG_OBJ_TYPE_RX);
    } else if (ID == 3) {
      CANMessageSet(CAN0_BASE, 3, &Message_Rx_3, MSG_OBJ_TYPE_RX);
    }
  }

void Fase_Balanceo() {
    LED("BLUE");
    // Se definen los setpoints de movimiento, 30 datos
    int hip[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                 11, 12, 13, 14, 15, 16, 17, 18, 19,
                 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30
                };
    int knee[] = {30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60,
                  57, 54, 51, 48, 45, 42, 39, 36, 33, 30,
                  27, 24, 21, 18, 15, 12, 9, 6, 3
                 };
    int ankle[] = { -20, -18, -16, -14, -12, -10, -8, -6, -4, -2,
                    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                    10, 9, 8, 7, 6, 5, 4, 3, 2, 1
                  };
  
    // Se mandan los set points de movimiento
    for (int i = 0; i < 30; i++) {
      delayMS(CAN_DELAY);
      motion_mode_command(1, hip[i], 0, 0.8, 1, 0);
      delayMS(CAN_DELAY);
      motion_mode_command(2, knee[i], 0, 0.7, 1, 0);
      delayMS(CAN_DELAY);
      motion_mode_command(3, ankle[i], 0, 0.6, 0.5, 0);
      if (walk_flag == 0) {
        stop_all_motors();
        return;
      }
    }
  }
  
  void Fase_ContactoInicial() {
    LED("CYAN");
  
    // Se definen los setpoints de movimiento, 30 datos
    int hip[] = {30, 29, 28, 27, 26, 25, 24, 23, 22, 21};
    int knee[] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 15};
    int ankle[] = {1, 0, -1, -2, -3, -5, -7, -8, -9, -10};
  
    // Se mandan los set points de movimiento
    for (int i = 0; i < 10; i++) {
      delayMS(CAN_DELAY);
      motion_mode_command(1, hip[i], 0, 0.8, 0, 0);
      delayMS(CAN_DELAY);
      motion_mode_command(2, knee[i], 0, 0.7, 0, 0);
      delayMS(CAN_DELAY);
      motion_mode_command(3, ankle[i], 0, 0.6, 0, 0);
      if (walk_flag == 0) {
        stop_all_motors();
        return;
      }
    }
  }
  
  void Fase_Apoyo() {
    LED("GREEN");
    // Se definen los setpoints de movimiento, 30 datos
    float hip[] = {20, 18, 17, 15, 14, 12, 11, 9, 8, 6,
                   5, 3, 2, 0, -1, -3, -4, -6, -7, -10
                  };
    float knee[] = {15, 14, 14, 13, 12, 12, 11, 10, 10,
                    9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 0
                   };
    float ankle[] = { -10, -8, -6, -4, -2, 0, 2, 4, 6,
                      8, 10, 12, 14, 15, 15, 15, 15, 15, 15, 15
                    };
  
    // Se mandan los set points de movimiento
    for (int i = 0; i < 20; i++) {
      delayMS(CAN_DELAY);
      motion_mode_command(1, hip[i], 0, 0.8, 0, 0);
      delayMS(CAN_DELAY);
      motion_mode_command(2, knee[i], 0, 0.7, 0, 0);
      delayMS(CAN_DELAY);
      motion_mode_command(3, ankle[i], 0, 0.6, 0, 0);
      if (walk_flag == 0) {
        stop_all_motors();
        return;
      }
    }
  }
  
  void Fase_PreBalanceo() {
    LED("RED");
    // Se definen los setpoints de movimiento, 20 datos
    float hip[] = { -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5,
                    -5, -4, -4, -3, -3, -2, -2, -1, 0
                  };
    float knee[] = {1, 3, 5, 7, 9, 10,
                    11, 13, 14, 16, 18, 19,
                    20, 21, 23, 24, 26, 28, 29, 30
                   };
    float ankle[] = {15, 13, 14, 12, 10, 7, 5, 2, 0, -3, -5,
                     -8, -10, -13, -15, -16, -17, -18, -19, -20
                    };
  
    // Se mandan los set points de movimiento
    for (int i = 0; i < 20; i++) {
      delayMS(CAN_DELAY);
      motion_mode_command(1, hip[i], 0, 0.8, 0, 0);
      delayMS(CAN_DELAY);
      motion_mode_command(2, knee[i], 0, 0.7, 0, 0);
      delayMS(CAN_DELAY);
      motion_mode_command(3, ankle[i], 0, 0.6, 0, 0);
      if (walk_flag == 0) {
        stop_all_motors();
        return;
      }
    }
  }

  void gait_simulation() {
    Fase_Balanceo();
    delayMS(2000);
    Fase_ContactoInicial();
    delayMS(2000);
    Fase_Apoyo();
    delayMS(2000);
    Fase_PreBalanceo();
    delayMS(2000);
    LED("OFF");
  }

  void set_absolute_position(int8_t ID, int32_t position_ref, int16_t max_speed) {
    // Función para establecer la posición absoluta
  
    // Objetos para la comunicación CAN
    uint8_t CAN_data_TX[8u];
    commandCAN = 0xA4; // comando de lectura
  
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
    send_cmd(ID, CAN_data_TX);
  }

  void split32bits(int32_t number, uint8_t *byteArray) {
    // Función para dividir una variable de 32 bits en 4 bytes
    byteArray[0] = (number >> 24) & 0xFF;  // byte más significativo
    byteArray[1] = (number >> 16) & 0xFF;
    byteArray[2] = (number >> 8) & 0xFF;
    byteArray[3] = number & 0xFF;  // byte menos significativo
  }
  
  void split16bits(int16_t number, uint8_t *byteArray) {
    // Función para dividir una variable de 16 bits en 2 bytes
    byteArray[0] = (number >> 8) & 0xFF; // byte más significativo
    byteArray[1] = number & 0xFF;  // byte menos significativo
  }

  void set_speed(int8_t ID, int64_t speed_ref) {
    // Función para establecer la velocidad
  
    // Objetos para la comunicación CAN
    uint8_t CAN_data_TX[8u];
    commandCAN = 0xA2; // comando de lectura
  
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
  
    send_cmd(ID, CAN_data_TX);
  }

  void loop() {
      // Lectura de comandos desde Serial
      
      if (Serial.available() > 0) { 
          char incomingByte = Serial.read();
          if (incomingByte == '1') { //Comando 1, todos los motores se mueven al angulo 0
              walk_flag = 1;
              gait_phase = 1;
              motion_mode_command(1, 0, 0, 0.8, 0, 0);
              delayMS(CAN_DELAY);
              motion_mode_command(2, 0, 0, 0.01, 0.1, 0);
              delayMS(CAN_DELAY);
              motion_mode_command(3, 0, 0, 6, 0.3, 0);
              delayMS(CAN_DELAY);
              Serial.println("0 grados");
          }
          else if (incomingByte == '2') { //Comando 2, todos los motores se mueven al angulo 90
              motion_mode_command(1, 90, 0, 0.8, 0, 0);
              delayMS(CAN_DELAY);
              motion_mode_command(2, 90, 0, 0.01, 0.1, 0);
              delayMS(CAN_DELAY);
              motion_mode_command(3, 90, 0, 6, 0.3, 0);
              delayMS(CAN_DELAY);
              Serial.println("90 grados");
          }
          else if (incomingByte == '3') { //Comando 3, todos los motores se mueven al angulo 180
            motion_mode_command(1, 180, 0, 0.8, 0, 0);
            delayMS(CAN_DELAY);
            motion_mode_command(2, 180, 0, 0.01, 0.1, 0);
            delayMS(CAN_DELAY);
            motion_mode_command(3, 180, 0, 6, 0.3, 0);
            delayMS(CAN_DELAY);
            Serial.println("180 grados");
          }
          else if (incomingByte == '4') { //Comando 4, reset para los motores, permite cambiar de modo servo a motion mode y vice versa
              walk_flag = 0;
              gait_phase = 0;
              reset_all_motors();
          }
          else if (incomingByte == '5') { //Comando 5, activa la secuencia de gait_simulation
              walk_flag = 1;
              gait_simulation();
          }
          else if (incomingByte == '6') {
              set_absolute_position(1, 12, max_speed);  //Comando 6, mueve todos los motores a la posicion 12 a la velocidad maxima en modo servo
              delayMS(CAN_DELAY);
              set_absolute_position(2, 12, max_speed);
              delayMS(CAN_DELAY);
              set_absolute_position(3, 12, max_speed);
              delayMS(CAN_DELAY);
          }
          else if (incomingByte == '7') {   //Comando 7, mueve los motores con una velocidad constante en modo servo
              set_speed(1, 12);
              delayMS(CAN_DELAY);
              set_speed(2, 12);
              delayMS(CAN_DELAY);
              set_speed(3, 12);  
              delayMS(CAN_DELAY);          
        }
      }    
  }

  
