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
void setup() {
  #define RED_LED GPIO_PIN_1 // Output to the onboard red led on the Tiva C development board. Pin is PF_1.
  #define BLUE_LED GPIO_PIN_2 // Output to the onboard blue led on the Tiva C development board. Pin is PF_2.
  #define GREEN_LED GPIO_PIN_3 // Output to the onboard green led on the Tiva C development board. Pin is PF_3.
  // Enable GPIO ports F, A, D, E and wait for them to be ready.
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Leds are in GPIOF.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Serial port is in GPIOA.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // H-Bridge control could be in GPIOD.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Analog inputs are in GPIOE.
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {}
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}
  
  // Configure the following pins as digital outputs.
  
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);
}

void loop() {
  GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, GREEN_LED|BLUE_LED);
  delay(500);
  GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, GREEN_LED|RED_LED);
  delay(500);
  GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, RED_LED|BLUE_LED);
  delay(500);
}
