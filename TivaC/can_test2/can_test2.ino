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
uint8_t CANBUSSend[8u];
uint8_t CANBUSReceive[8u];
uint8_t inByte = 0;
bool doControlFlag = 0;

tCANMsgObject sMsgObjectRx;
tCANMsgObject sMsgObjectTx;

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

void loop() {
    if (doControlFlag) {
        doControlFlag = false;
        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, RED_LED); // CPU usage measurement

        // Trigger ADC conversion
        ADCProcessorTrigger(ADC0_BASE, 0);
        while (!ADCIntStatus(ADC0_BASE, 0, false)) {}
        ADCIntClear(ADC0_BASE, 0);
        ADCSequenceDataGet(ADC0_BASE, 0, &intensity);

        // Set up CAN receive message object
        sMsgObjectRx.ui32MsgID = 0x241u;
        sMsgObjectRx.ui32MsgIDMask = 0xFFFFFFFFu; // Reads all messages
        sMsgObjectRx.ui32MsgLen = 8u;
        sMsgObjectRx.pui8MsgData = CANBUSReceive;
        CANMessageSet(CAN0_BASE, 2, &sMsgObjectRx, MSG_OBJ_TYPE_RXTX_REMOTE);

        // Prepare CAN message to send
        CANBUSSend[0] = 0x30;
        CANBUSSend[1] = 0x00;
        CANBUSSend[2] = 0x00;
        CANBUSSend[3] = 0x00;
        CANBUSSend[4] = 0x00;
        CANBUSSend[5] = 0x00;
        CANBUSSend[6] = 0x00;
        CANBUSSend[7] = 0x00;

        sMsgObjectTx.ui32MsgID = 0x141;
        sMsgObjectTx.ui32MsgIDMask = 0xFFFFFFFF;
        sMsgObjectTx.ui32MsgLen = 8u;
        sMsgObjectTx.pui8MsgData = CANBUSSend;
        CANMessageSet(CAN0_BASE, 1, &sMsgObjectTx, MSG_OBJ_TYPE_TX);

        // Wait for the message to be transmitted
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

        while (UARTCharsAvail(UART0_BASE)) {
            inByte = UARTCharGet(UART0_BASE);
            if (inByte == '$') {
                uint8_t sendBuffer[2];
                memcpy(sendBuffer, &intensity, sizeof(intensity));
                UARTCharPut(UART0_BASE, sendBuffer[1]);
                UARTCharPut(UART0_BASE, sendBuffer[0]);
            }
        }

        GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, 0);
    }
}
