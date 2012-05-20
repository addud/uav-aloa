#ifndef __I2C_H
#define __I2C_H

#include "stm32f10x.h"

void I2CInit(void);
uint16_t I2CSendCommand(uint8_t device_address, uint8_t command_address, uint8_t command);
uint16_t I2CReadData(uint8_t device_address, uint8_t register_address);

#endif
