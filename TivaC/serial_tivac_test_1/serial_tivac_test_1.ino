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

#define UART5_RX_PIN GPIO_PIN_4
#define UART5_TX_PIN GPIO_PIN_5

tCANMsgObject sMsgObjectRx;
tCANMsgObject sMsgObjectTx_PIDvalues;
tCANMsgObject sMsgObjectTx_Control;
tCANMsgObject sMsgObjectTx_Stop;

int i = 0;

void setup() {
    Serial.begin(115200);

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

    // Configura el reloj del UART5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  
    // Configura los pines UART5 para RX y TX
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    GPIOPinTypeUART(GPIO_PORTE_BASE, UART5_RX_PIN | UART5_TX_PIN);
  
    // Configura el UART5
    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    
    // Mensaje de depuración en el monitor serial
    Serial.println("Tiva C inicializado.");
    Serial.println("Configurando UART5...");
}

void loop() {
  // Verifica si hay datos disponibles en el UART5
  if (UARTCharsAvail(UART5_BASE)) {
    // Lee un carácter del UART5
    char receivedChar = UARTCharGet(UART5_BASE);
    
    // Imprime el carácter recibido en el monitor serial
    Serial.print(receivedChar);
    
    // Mensaje de depuración en el monitor serial
    Serial.println("Caracter recibido.");
  }
  
  // Espera 100 ms antes de la siguiente iteración
  delay(100);
}
