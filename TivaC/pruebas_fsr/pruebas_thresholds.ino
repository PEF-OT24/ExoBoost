#define PART_TM4C123GH6PM
#include <stdint.h>
#include <stdbool.h>
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
#include "driverlib/systick.h"

// Definimos los pines correspondientes a los LEDs
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

// ------------------- Variables -----------------------
// Declare a byte array of two elements, used to store the data to be sent
uint8_t sendBuffer[5], inByte = 0, Toe, Left, Right, Heel, GaitPhase, Thresh;
uint8_t TH_toe, TH_left, TH_right, TH_heel;
uint32_t adcValues[4];
uint32_t dutyCycle = 0;
uint8_t thresholds[4];
int cont = 0;

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

void ConfigADC(){
  // Enable ADC0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
  // Wait for the ADC0 module to be ready
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

  // Configure the following pins as ADC inputs.
  GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
  
  // Configurar el ADC con 4 canales
  ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

  // Configura la secuencia de lectura de 4 canales
  ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH0); // toe
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH1); // left
  ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2); // right
  ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);  // heel, interrupción
  
  // Enable sequence 1
  ADCSequenceEnable(ADC0_BASE, 1);
}

void ReadADC(void){
  // Ejecutar Conversión ADC
  ADCProcessorTrigger(ADC0_BASE, 1);
  
  // Esperar por lectura
  while (!ADCIntStatus(ADC0_BASE, 1, false));
  
  // Limpiar el buffer
  ADCIntClear(ADC0_BASE, 1);
  
  // Leer valores
  ADCSequenceDataGet(ADC0_BASE, 1, adcValues);
  
  // Procesamiento de datos 
  sendBuffer[0] = map(adcValues[0], 0, 4095, 0, 255); // toe
  sendBuffer[1] = map(adcValues[1], 0, 4095, 0, 255); // left
  sendBuffer[2] = map(adcValues[2], 0, 4095, 0, 255); // right
  sendBuffer[3] = map(adcValues[3], 0, 4095, 0, 255); // heel

  /*
  // Mandar los datos al HMI
  if (Serial.available() > 0) {
    inByte = Serial.read();
    if( inByte == '#'){ 
      Serial.write("Y", 1);
      Serial.write(sendBuffer, 4);
    }
  }
  */

  /*
  // Impresión de calibración
  Serial.print("Toe: "); Serial.print(thresholds[0]);
  Serial.print(" Left: "); Serial.print(thresholds[1]);
  Serial.print(" Right: "); Serial.print(thresholds[2]);
  Serial.print(" Heel: "); Serial.println(thresholds[3]);Serial.println(); // Salto de línea
  */

  // Detección de thresholds
  for(int i = 0; i<4; i++){
    if(i == 0){Serial.print("Toe: ");}
    if(i == 1){Serial.print("Left: ");}
    if(i == 2){Serial.print("Right: ");}
    if(i == 3){Serial.print("Heel: ");}
    if(sendBuffer[i] > thresholds[i]){Serial.print("SI");}else{Serial.print("NO");}
    Serial.print(" ");
  }
  Serial.println();
}

void CalibrarADC(){
  GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, BLUE_LED);
  uint8_t toe_max = 0, left_max = 0, right_max = 0, heel_max = 0;
  uint8_t THS[4];
  
  for(int i = 0; i < 100; i++){ // proceso para recuperar 100 datos y realizar calibración
    int8_t toe_value, left_value, right_value, heel_value;
    
    // Ejecutar Conversión ADC
    ADCProcessorTrigger(ADC0_BASE, 1);
    
    // Esperar por lectura
    while (!ADCIntStatus(ADC0_BASE, 1, false));
    
    // Limpiar el buffer
    ADCIntClear(ADC0_BASE, 1);
    
    // Leer valores
    ADCSequenceDataGet(ADC0_BASE, 1, adcValues);
    
    // Procesamiento de datos 
    toe_value = map(adcValues[0], 0, 4095, 0, 255); // toe
    left_value = map(adcValues[1], 0, 4095, 0, 255); // left
    right_value = map(adcValues[2], 0, 4095, 0, 255); // right
    heel_value = map(adcValues[3], 0, 4095, 0, 255); // heel

    // Detección de máximos
    if(toe_value > toe_max){toe_max = toe_value;}
    if(left_value > left_max){left_max = left_value;}
    if(right_value > right_max){right_max = right_value;}
    if(heel_value > heel_max){heel_max = heel_value;}

    delayMS(10); // ligero delay en ms
  }

  // Se le agrega un offset
  toe_max = toe_max;
  left_max = left_max;
  right_max = right_max;
  heel_max = heel_max;

  // Se guardan los thresholds y se devuelven
  thresholds[0] = toe_max;
  thresholds[1] = left_max;
  thresholds[2] = right_max;
  thresholds[3] = heel_max;

  /*
  // Mandar los datos al HMI
  if (Serial.available() > 0) {
    inByte = Serial.read();
    if( inByte == '#'){ 
      Serial.write("X", 1);
      Serial.write(thresholds, 4);
    }
  }
  */
  
  GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, 0);
}

void setup() {
  Serial.begin(9600);
  // Establecer reloj a 80 MHz. 
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  // Se habilita el puerto F
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  // Configuramos los pines del LED como salida
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED);
  
  // Habilita interrupciones
  IntMasterEnable();
  ConfigADC();

  // Calibración inicial
  CalibrarADC();
}

void loop(){
  // Hacer lecturas y mandarlas
  ReadADC();
}
