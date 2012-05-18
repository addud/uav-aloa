/**
  ******************************************************************************
  * @file    TIM/InputCapture/stm32f10x_it.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

void initPPM(void);

int16_t getNick(void);
int16_t getRoll(void);
int16_t getGas(void);
int16_t getYaw(void);
int16_t getPoti1(void);
int16_t getPoti2(void);
int16_t getPoti3(void);
int16_t getPoti4(void);
int16_t getPoti5(void);
int16_t getPoti6(void);
int16_t getPoti7(void);
int16_t getPoti8(void);
void setNick(int16_t value);
void setRoll(int16_t value);
void setGas(int16_t value);
void setYaw(int16_t value);
void setPoti1(int16_t value);
void setPoti2(int16_t value);
void setPoti3(int16_t value);
void setPoti4(int16_t value);
void setPoti5(int16_t value);
void setPoti6(int16_t value);
void setPoti7(int16_t value);
void setPoti8(int16_t value);

#endif /* __STM32F10x_IT_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
