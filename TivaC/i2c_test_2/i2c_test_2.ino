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

uint32_t i2c_status;
uint32_t rec_data = 1;
#define SLAVE_ADDRESS 0x55 // Define the I2C slave address
volatile uint8_t receivedData[999];  // Array to store received data
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
    IntMasterEnable();
    IntEnable(INT_I2C0);
    // Habilitar módulo I2C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)){Serial.println("Esperando I2C0");}
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    // Configurar la multiplexación de pines para las funciones I2C1 en los pines PA6 (SCL) y PA7 (SDA)
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    // Configuración de esclavo
    I2CSlaveInit(I2C0_BASE, 85); // 85 en decimal es 0x55 en hex
    I2CSlaveEnable(I2C0_BASE);    

    Serial.println("Módulo inicializado");
}

/*
void loop(){
  // Main loop vacío
  i2c_status = I2CSlaveStatus(I2C0_BASE);
  
  // rec_data = I2CSlaveDataGet(I2C0_BASE); // Lectura de los datos recibidos
  Serial.println(i2c_status);
  
  delay(500); 
}
*/

void loop() {
  // Imprimir datos recibidos si están disponibles
  interruptI2C();
  /*
  if (dataCounter > 0) {
    Serial.print("Datos recibidos: ");
    for (int i = 0; i < dataCounter; i++) {
      Serial.print(receivedData[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    dataCounter = 0;  // Reiniciar el contador de datos
  }
  */

  delay(1000);  // Esperar antes de la siguiente iteración del loop
}

// Manejador de interrupciones I2C
void interruptI2C() {
  uint32_t status = I2CSlaveIntStatus(I2C0_BASE, false);

  // Limpiar la bandera de interrupción I2C
  I2CSlaveIntClear(I2C0_BASE);

  if (status & I2C_SLAVE_INT_DATA) {
    // Leer datos del master
    receivedData[dataCounter] = I2CSlaveDataGet(I2C0_BASE);
    char data = I2CSlaveDataGet(I2C0_BASE);
    Serial.print("Datos recibidos: ");
    Serial.println(data);
    dataCounter++;
  }
}
