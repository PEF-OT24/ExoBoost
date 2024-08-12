#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"

void setup() {
  SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE,GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE,GPIO_PIN_3);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)){}
  I2CMasterInitExpClk(I2C0_BASE,SysCtlClockGet(),true);
  I2CMasterSlaveAddrSet(I2C0_BASE, 0x55, false);
}

void loop() {
  I2CMasterDataPut(I2C0_BASE, 'A');

  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
  SysCtlDelay(2000000); 
}