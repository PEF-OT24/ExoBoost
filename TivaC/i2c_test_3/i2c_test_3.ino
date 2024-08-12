// Código para comunicación I2C en la Tiva como esclavo usando su API nativa

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
#include "driverlib/i2c.h" // Se agrega la librería de I2C

uint32_t intensity = 0;
uint8_t CANBUSSend_PIDvalues[8u];
uint8_t CANBUSSend_Control[8u];
uint8_t CANBUSSend_Stop[8u];
uint8_t CANBUSReceive[8u];
uint8_t inByte = 0;
bool doControlFlag = 0;

#define I2C_SLAVE_ADDRESS 0x55  // Slave address
#define BUFFER_SIZE 32          // Buffer size for received data
char receivedData[BUFFER_SIZE];  // Buffer to store received data
uint8_t dataIndex = 0;           // Index to track the position in the buffer

uint32_t i2c_status;
uint32_t rec_data = 1;
volatile int dataCounter = 0;       // Counter for the number of bytes received

tCANMsgObject sMsgObjectRx;
tCANMsgObject sMsgObjectTx_PIDvalues;
tCANMsgObject sMsgObjectTx_Control;
tCANMsgObject sMsgObjectTx_Stop;

int i = 0;

void setup() {
    Serial.begin(9600);

    // -------- Habilitar periféricos -------- 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {Serial.println("Esperando puerto F");}
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED);

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {Serial.println("Esperando ADC");}
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    ADCSequenceEnable(ADC0_BASE, 0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {Serial.print("Esperando UART");}
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0)) {Serial.print("Esperando CAN0");}
    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 1000000u);
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    CANEnable(CAN0_BASE);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {Serial.println("Esperando puerto B");}
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Habilitar las interrupciones para el I2C0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the GPIO pins for I2C0 (PB2 - I2C0SCL, PB3 - I2C0SDA)
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    // Configure the I2C0 as a slave device
    I2CSlaveInit(I2C0_BASE, I2C_SLAVE_ADDRESS);
    
    memset(receivedData, 0, BUFFER_SIZE);

    Serial.println("Módulo inicializado");
}

void loop() {
    // Check if there is data available from the master
    if (I2CSlaveStatus(I2C0_BASE) & I2C_SLAVE_ACT_RREQ) {
        // Read the received byte
        char receivedByte = I2CSlaveDataGet(I2C0_BASE);

        // Store the received byte in the buffer
        if (dataIndex < BUFFER_SIZE - 1) {
            receivedData[dataIndex++] = receivedByte;
        }

        // If end of string (null character), reset index and print the message
        if (receivedByte == '\0' || dataIndex >= BUFFER_SIZE - 1) {
            Serial.print("Received: ");
            Serial.println(receivedData);
            dataIndex = 0;  // Reset buffer index
            memset(receivedData, 0, BUFFER_SIZE);  // Clear buffer
        }
    } else {
        // No data received, print a waiting message
        Serial.println("No data received");
        SysCtlDelay(SysCtlClockGet() / 10);  // Small delay for clarity in output
    }
}
