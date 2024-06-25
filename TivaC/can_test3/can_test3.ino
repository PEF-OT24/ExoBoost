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
uint8_t CANBUSSend_PIDvalues[8u];
uint8_t CANBUSSend_Control[8u];
uint8_t CANBUSSend_Stop[8u];
uint8_t CANBUSReceive[8u];
uint8_t inByte = 0;
bool doControlFlag = 0;

tCANMsgObject sMsgObjectRx;
tCANMsgObject sMsgObjectTx_PIDvalues;
tCANMsgObject sMsgObjectTx_Control;
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

    // Sends PID parameters
    SendParameters();
    SendSetPoint();
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
  // Define PID parameter values
  CANBUSSend_PIDvalues[0] = 0x32;
  CANBUSSend_PIDvalues[1] = 0x00;
  CANBUSSend_PIDvalues[2] = 0x64;
  CANBUSSend_PIDvalues[3] = 0x32;
  CANBUSSend_PIDvalues[4] = 0x64;
  CANBUSSend_PIDvalues[5] = 0x05;
  CANBUSSend_PIDvalues[6] = 0x64;
  CANBUSSend_PIDvalues[7] = 0x00;
  
  sMsgObjectTx_PIDvalues.ui32MsgID = 0x141;
  sMsgObjectTx_PIDvalues.ui32MsgIDMask = 0xFFFFFFFF;
  sMsgObjectTx_PIDvalues.ui32MsgLen = 8u;
  sMsgObjectTx_PIDvalues.pui8MsgData = CANBUSSend_PIDvalues;
  CANMessageSet(CAN0_BASE, 1, &sMsgObjectTx_PIDvalues, MSG_OBJ_TYPE_TX);
}

void SendSetPoint(){
  // Define PID parameter values
  CANBUSSend_Control[0] = 0xA2;
  CANBUSSend_Control[1] = 0x00;
  CANBUSSend_Control[2] = 0x00;
  CANBUSSend_Control[3] = 0x00;
  CANBUSSend_Control[4] = 0x00;
  CANBUSSend_Control[5] = 0x68;
  CANBUSSend_Control[6] = 0x01;
  CANBUSSend_Control[7] = 0x00;
  
  sMsgObjectTx_Control.ui32MsgID = 0x141;
  sMsgObjectTx_Control.ui32MsgIDMask = 0xFFFFFFFF;
  sMsgObjectTx_Control.ui32MsgLen = 8u;
  sMsgObjectTx_Control.pui8MsgData = CANBUSSend_Control;
  CANMessageSet(CAN0_BASE, 1, &sMsgObjectTx_Control, MSG_OBJ_TYPE_TX); 
}

void StopMotor(){
  // Define PID parameter values
  CANBUSSend_Stop[0] = 0x81;
  CANBUSSend_Stop[1] = 0x00;
  CANBUSSend_Stop[2] = 0x00;
  CANBUSSend_Stop[3] = 0x00;
  CANBUSSend_Stop[4] = 0x00;
  CANBUSSend_Stop[5] = 0x00;
  CANBUSSend_Stop[6] = 0x00;
  CANBUSSend_Stop[7] = 0x00;
  
  sMsgObjectTx_Stop.ui32MsgID = 0x141;
  sMsgObjectTx_Stop.ui32MsgIDMask = 0xFFFFFFFF;
  sMsgObjectTx_Stop.ui32MsgLen = 8u;
  sMsgObjectTx_Stop.pui8MsgData = CANBUSSend_Control;
  CANMessageSet(CAN0_BASE, 1, &sMsgObjectTx_Stop, MSG_OBJ_TYPE_TX); 
}

void loop() {
  // Wait for the message to be transmitted
  SendSetPoint();
  while (CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST)) {
    Serial.println("waiting");
    }
  
  // Get the received CAN message
  CANMessageGet(CAN0_BASE, 2, &sMsgObjectRx, false);
  
  // Print received CAN data to the serial monitor
  char buffer[50];
  sprintf(buffer, "Received: %02X %02X %02X %02X %02X %02X %02X %02X", 
          CANBUSReceive[0], CANBUSReceive[1], CANBUSReceive[2], CANBUSReceive[3], 
          CANBUSReceive[4], CANBUSReceive[5], CANBUSReceive[6], CANBUSReceive[7]);
  Serial.println(buffer);
}