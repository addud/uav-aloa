#ifndef __I2C_H
#define __I2C_H

#include "stm32f10x.h"


//I2C communication errors

#define IS_I2C_ERROR(X) ( ((X) >= (I2C_ERROR_START_VALUE)) ? true : false)

#define I2C_ERROR_START_VALUE													0xF000

#define I2C_TX_FLAG_BUSY_TIMEOUT											0xF000
#define I2C_TX_MASTER_MODE_TIMEOUT										0xF001
#define I2C_TX_FLAG_ADDR_TIMEOUT											0xF002
#define I2C_TX_MASTER_TRANSMITTER_TIMEOUT							0xF003
#define I2C_TX_BYTE_TRANSMITTED_TIMEOUT								0xF004
#define I2C_TX_CLEAR_STOP_TIMEOUT											0xF005
#define I2C_RX_FLAG_BUSY_TIMEOUT											0xF006
#define I2C_RX_MASTER_MODE_TIMEOUT										0xF007
#define I2C_RX_FLAG_ADDR_TIMEOUT											0xF008
#define I2C_RX_MASTER_TRANSMITTER_TIMEOUT							0xF009
#define I2C_RX_BYTE_TRANSMITTED_TIMEOUT								0xF00A
#define I2C_RX_CLEAR_STOP_TIMEOUT											0xF00B
#define I2C_RX_FLAG_BTF_TIMEOUT												0xF00C
#define I2C_RX_FLAG_RXNE_TIMEOUT											0xF00D

void I2CInit(void);
uint16_t I2CSendCommand(uint8_t device_address, uint8_t command_address, uint8_t command);
uint16_t I2CReadData(uint8_t device_address, uint8_t register_address);

#endif
