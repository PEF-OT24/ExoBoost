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

// ------------------- Variables -----------------------
// Declare a byte array of two elements, used to store the data to be sent
uint8_t sendBuffer[5], inByte = 0, Toe, Left, Right, Heel, GaitPhase, Thresh;
uint32_t adcValues[4];
uint32_t dutyCycle = 0;

void ConfigADC(){
  // Enable ADC0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
    // Wait for the ADC0 module to be ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    // Configure the following pins as ADC inputs.
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    
    // Configure ADC sequence 1 (4 channels)
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    
    // Configure each step of the sequence for 4 channels
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);  // Last step with interrupt
    
    // Enable sequence 1
    ADCSequenceEnable(ADC0_BASE, 1);
}

void ReadADC(void){
    // Trigger ADC conversion
    ADCProcessorTrigger(ADC0_BASE, 1);
    
    // Wait for conversion to complete
    while (!ADCIntStatus(ADC0_BASE, 1, false));
    
    // Clear ADC interrupt flag
    ADCIntClear(ADC0_BASE, 1);
    
    // Read ADC values
    ADCSequenceDataGet(ADC0_BASE, 1, adcValues);
}

void setup() {
  Serial.begin(9600);
  // Set processorclockat 80Mhz. 
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
  
  // Enable interrupts on the system.
  IntMasterEnable();
  ConfigADC();
}

void loop(){
  ReadADC();
  sendBuffer[0] = map(adcValues[0], 0, 4095, 0, 255);
  sendBuffer[1] = map(adcValues[1], 0, 4095, 0, 255);
  sendBuffer[2] = map(adcValues[2], 0, 4095, 0, 255);
  sendBuffer[3] = map(adcValues[3], 0, 4095, 0, 255); 
  // Wait until a new character arrives to the receive buffer
  if (Serial.available() > 0) {
    inByte = Serial.read();
    if( inByte == '#'){ 
      Serial.write(sendBuffer, 4);
    }
  }
}