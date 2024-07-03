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
#include "Wire.h"

// LED Definitions
#define RED_LED GPIO_PIN_1
#define BLUE_LED GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

// Define CAN_INT_INTID_STATUS if not defined
#ifndef CAN_INT_INTID_STATUS
#define CAN_INT_INTID_STATUS 0x8000
#endif

uint32_t intensity = 0;
uint8_t CANBUSSend_Stop[8u];
uint8_t CANBUSReceive[8u];
uint8_t inByte = 0;
bool doControlFlag = 0;

tCANMsgObject sMsgObjectRx;
tCANMsgObject sMsgObjectTx_PIDvalues;
tCANMsgObject sMsgObjectTx_Stop;

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

    IntMasterEnable();
    SendParameters();
    //set_speed(1, 380, true);
    //set_absolute_position(1, 90, 0, true);
}

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

void ISRSysTick(void) {
    doControlFlag = true;
}

void CAN0IntHandler(void) {
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
        sMsgObjectRx.pui8MsgData = CANBUSReceive;
        CANMessageGet(CAN0_BASE, 2, &sMsgObjectRx, false);
    }
}

void SendParameters(){
  // Se define la variable que almacena el valor
  uint8_t CANBUSSend_PIDvalues[8u];

  // Define PID parameter values
  CANBUSSend_PIDvalues[0] = 0x32; 
  CANBUSSend_PIDvalues[1] = 0x00;
  CANBUSSend_PIDvalues[2] = 0x64;
  CANBUSSend_PIDvalues[3] = 0x32;
  CANBUSSend_PIDvalues[4] = 0x64;
  CANBUSSend_PIDvalues[5] = 0x32;
  CANBUSSend_PIDvalues[6] = 0x64;
  CANBUSSend_PIDvalues[7] = 0x32;
  
  sMsgObjectTx_PIDvalues.ui32MsgID = 0x141;
  sMsgObjectTx_PIDvalues.ui32MsgIDMask = 0xFFFFFFFF;
  sMsgObjectTx_PIDvalues.ui32MsgLen = 8u;
  sMsgObjectTx_PIDvalues.pui8MsgData = CANBUSSend_PIDvalues;
  CANMessageSet(CAN0_BASE, 1, &sMsgObjectTx_PIDvalues, MSG_OBJ_TYPE_TX);
}

void set_speed(int8_t ID, int64_t speed_ref, bool show){
  
  // Objetos para la comunicación CAN
  tCANMsgObject Message_Tx;
  tCANMsgObject Message_Rx;
  uint8_t CAN_data_TX[8u];
  uint8_t CAN_data_RX[8u];

  // Escala el valor a enviar 0.01 dps/LSB
  int32_t sp = speed_ref * 100;

  //if (sp > 2147483647){
  //  return;
  //}

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

  // Define el mensaje para mandar por CAN
  Message_Tx.ui32MsgID = 0x140 + ID;
  Message_Tx.ui32MsgIDMask = 0xFFFFFFFF;
  Message_Tx.ui32MsgLen = 8u;
  Message_Tx.pui8MsgData = CAN_data_TX;

  // Define el mensaje para leer por CAN
  Message_Rx.ui32MsgID = 0x240 + ID;
  Message_Rx.ui32MsgIDMask = 0xFFFFFFFF; // Lee todos los mensajes
  Message_Rx.ui32MsgLen = 8u;
  Message_Rx.pui8MsgData = CAN_data_RX;

  // Envío por CAN
  CANMessageSet(CAN0_BASE, 1, &Message_Tx, MSG_OBJ_TYPE_TX); 

  // Lee el mensaje de respuesta
  CANMessageSet(CAN0_BASE, 2, &Message_Rx, MSG_OBJ_TYPE_RXTX_REMOTE);

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

void set_absolute_position(int8_t ID, int64_t position_ref, int64_t max_speed, bool show){ // NO FUNCIONA
  
  // Objetos para la comunicación CAN
  tCANMsgObject Message_Tx;
  tCANMsgObject Message_Rx;
  uint8_t CAN_data_TX[8u];
  uint8_t CAN_data_RX[8u];

  // Escala el valor a enviar 0.01 dps/LSB
  int32_t sp = position_ref * 100;

  //if (sp > 2147483647){
  //  return;
  //}

  uint8_t byteArray_speed[2];
  uint8_t byteArray_position[4];

  // Split the number into 4 bytes
  split32bits(sp, byteArray_position);
  split16bits(max_speed, byteArray_speed);
  
  // Define el setpoint de la velocidad
  CAN_data_TX[0] = 0xA4;
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0;
  CAN_data_TX[3] = 0;
  CAN_data_TX[4] = byteArray_position[3];
  CAN_data_TX[5] = byteArray_position[2];
  CAN_data_TX[6] = byteArray_position[1];
  CAN_data_TX[7] = byteArray_position[0];

  // Define el mensaje para mandar por CAN
  Message_Tx.ui32MsgID = 0x140 + ID;
  Message_Tx.ui32MsgIDMask = 0xFFFFFFFF;
  Message_Tx.ui32MsgLen = 8u;
  Message_Tx.pui8MsgData = CAN_data_TX;

  // Define el mensaje para leer por CAN
  Message_Rx.ui32MsgID = 0x240 + ID;
  Message_Rx.ui32MsgIDMask = 0xFFFFFFFF; // Lee todos los mensajes
  Message_Rx.ui32MsgLen = 8u;
  Message_Rx.pui8MsgData = CAN_data_RX;

  // Envío por CAN
  CANMessageSet(CAN0_BASE, 1, &Message_Tx, MSG_OBJ_TYPE_TX); 

  // Lee el mensaje de respuesta
  CANMessageSet(CAN0_BASE, 2, &Message_Rx, MSG_OBJ_TYPE_RXTX_REMOTE);

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

void stop_motor(int8_t ID){
  
  // Objetos para la comunicación CAN
  tCANMsgObject Message_Tx;
  uint8_t CAN_data_TX[8u];

  // Reset del motor
  CAN_data_TX[0] = 0x81; // con 80 se apaga el motor
  CAN_data_TX[1] = 0x00;
  CAN_data_TX[2] = 0x00;
  CAN_data_TX[3] = 0x00;
  CAN_data_TX[4] = 0x00;
  CAN_data_TX[5] = 0x00;
  CAN_data_TX[6] = 0x00;
  CAN_data_TX[7] = 0x00;

  // Define el mensaje para mandar por CAN
  Message_Tx.ui32MsgID = 0x140 + ID;
  Message_Tx.ui32MsgIDMask = 0xFFFFFFFF;
  Message_Tx.ui32MsgLen = 8u;
  Message_Tx.pui8MsgData = CAN_data_TX;
  
  // Envío por CAN
  CANMessageSet(CAN0_BASE, 1, &Message_Tx, MSG_OBJ_TYPE_TX); 

}
void loop() {
  // Wait for the message to be transmitted
  stop_motor(1);
  //set_absolute_position(1, 90, 0, true);
}
