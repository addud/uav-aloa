
#include "stm32f10x_it.h"
#include "I2C.h"

#define I2C_SPEED               300000

#define I2C_TIMEOUT							0x1000

/*
To start a measurement a write command:

1. Send a start sequence
2. Send 0xE0 ( I2C address of the SRF02 with the R/W bit low (even address - write))
3. Send 0x00 (Internal address of the command register)
4. Send 0x51 (The command to start the SRF02 ranging in cm)
5. Send the stop sequence.

To read the range:

1. Send a start sequence
2. Send 0xE0 ( I2C address of the SRF02 with the R/W bit low (even address - write))
3. Send 0x02 (Internal address of the first distance register)
4. Send a start sequence again (repeated start)
5. Send 0xE1 ( I2C address of the SRF02 with the R/W bit high (odd address - read))
6. Read the two data bytes, high and low from SRF02
7. Send the stop sequence. 

*/

void I2CInit(void)
{ 
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
    
  /*!< sEE_I2C_SCL_GPIO_CLK and sEE_I2C_SDA_GPIO_CLK Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB2Periph_GPIOB, ENABLE);

  /*!< sEE_I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    
  /*!< GPIO configuration */  
  /*!< Configure I2C pins: SDA -- PB6*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /*!< Configure I2C pins: SDA -- PB7*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);   
  
  /*!< I2C configuration */
  /* sEE_I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
  
  /* sEE_I2C Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);
  /* Apply sEE_I2C configuration after enabling it */
  I2C_Init(I2C1, &I2C_InitStructure);
   
}


uint16_t I2Ctimeout() {
	 return 1;
}

uint16_t I2CSendCommand(uint8_t device_address, uint8_t command_address, uint8_t command)
{ 
	uint16_t timeout;
  
  /*!< While the bus is busy */
  timeout = I2C_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }

	//1. Send a start sequence
  
  /*!< Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /*!< Test on EV5 and clear it */
  timeout = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }

	//2. Send 0xE0 ( I2C address of the SRF02 with the R/W bit low (even address - write))
  
  /*!< Send Sonar address for write */
  timeout = I2C_TIMEOUT;
  I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Transmitter);

  /* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
  timeout = I2C_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) == RESET)
  {
//    if((timeout--) == 0) return I2Ctimeout();
  } 

  /*!< Test on EV6 and clear it */
  timeout = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }

	//3. Send 0x00 (Internal address of the command register)

  /*!< Send the Sonar's internal register address to write to */
  I2C_SendData(I2C1, command_address);

  /*!< Test on EV8 and clear it */
  timeout = I2C_TIMEOUT;  
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
//    if((timeout--) == 0) return I2Ctimeout();
  } 

	//4. Send 0x51 (The command to start the SRF02 ranging in cm)

  /*!< Send the command to the sonar */
  I2C_SendData(I2C1, command);

  /*!< Test on EV8 and clear it */
  timeout = I2C_TIMEOUT;  
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
//    if((timeout--) == 0) return I2Ctimeout();
  } 

	//5. Send the stop sequence.

	/*!< Send STOP Condition */
  I2C_GenerateSTOP(I2C1, ENABLE);  

	/* Wait to make sure that STOP control bit has been cleared */
  timeout = I2C_TIMEOUT;
  while(I2C1->CR1 & I2C_CR1_STOP)
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }  

  /* If all operations OK, return OK */
  return 1;
}


uint16_t I2CReadData(uint8_t device_address, uint8_t register_address) {
	uint16_t timeout;
	static uint16_t	data=0;

 /*!< While the bus is busy */
  timeout = I2C_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }

	//1. Send a start sequence
  
  /*!< Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /*!< Test on EV5 and clear it */
  timeout = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }

	//2. Send 0xE0 ( I2C address of the SRF02 with the R/W bit low (even address - write)) 
  
  /*!< Send Sonar address for write */
  timeout = I2C_TIMEOUT;
  I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Transmitter);

  /* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
  timeout = I2C_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) == RESET)
  {
//    if((timeout--) == 0) return I2Ctimeout();
  } 

  /*!< Test on EV6 and clear it */
  timeout = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }

	//3. Send 0x02 (Internal address of the first distance register)

  /*!< Send the sonar internal register address to read from */
  I2C_SendData(I2C1, register_address);

  /*!< Test on EV8 and clear it */
  timeout = I2C_TIMEOUT;  
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
//    if((timeout--) == 0) return I2Ctimeout();
  } 

	I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Next);
  
	//4. Send a start sequence again (repeated start)

  /*!< Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /*!< Test on EV5 and clear it */
  timeout = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }
  
	//5. Send 0xE1 ( I2C address of the SRF02 with the R/W bit high (odd address - read))

  /*!< Send sonar address for read */
  I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Receiver);  

  /* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
  timeout = I2C_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) == RESET)
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }  

  /* Call User callback for critical section start (should typically disable interrupts) */
  __disable_irq();   
  
  /* Clear ADDR register by reading SR1 then SR2 register (SR1 has already been read) */
  (void)I2C1->SR2;

  /*!< Disable Acknowledgement */
  I2C_AcknowledgeConfig(I2C1, DISABLE);
	
	/* Call User callback for critical section end (should typically re-enable interrupts) */
  __enable_irq();   
 
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF) == RESET)
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }

	//6. Read the two data bytes, high and low from SRF02
 
   /* Wait for the byte to be received */
  timeout = I2C_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }
  
  /*!< Read first data byte from sonar  */
  data = I2C_ReceiveData(I2C1) << 8; 
  
  /* Wait for the byte to be received */
  timeout = I2C_TIMEOUT;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }
  
  /*!< Read second data byte from sonar */
  data |= I2C_ReceiveData(I2C1); 

	//7. Send the stop sequence.
	
	__disable_irq();
	  
  /*!< Send STOP Condition */
  I2C_GenerateSTOP(I2C1, ENABLE);
 
  /* Call User callback for critical section end (should typically re-enable interrupts) */
  __enable_irq(); 
	
  /* Wait to make sure that STOP control bit has been cleared */
  timeout = I2C_TIMEOUT;
  while(I2C1->CR1 & I2C_CR1_STOP)
  {
//    if((timeout--) == 0) return I2Ctimeout();
  }  
  
  /*!< Re-Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);  

	I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);

	return data;  
}

