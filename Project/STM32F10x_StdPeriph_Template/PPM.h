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
#include <stdbool.h>

//PPM values
#define PPM_MIN_VALUE							(PPM_MIN_HIGH - PPM_NEUTRAL_HIGH)
#define PPM_MAX_VALUE							(PPM_MAX_HIGH - PPM_NEUTRAL_HIGH)
#define PPM_NEUTRAL_VALUE					0
#define PPM_SWITCH_VALUE_JITTER		50

typedef enum {
	SW_ON = 0x01,
	SW_OFF = 0x00,
	SW_NEUTRAL = 0x02,
	SW_UNKNOWN = 0x03
} ppm_switch_values_t;

void initPPM(void);

int16_t getNick(void);
int16_t getRoll(void);
int16_t getGas(void);
int16_t getYaw(void);
ppm_switch_values_t getPoti1(void);
ppm_switch_values_t getPoti2(void);
int16_t getPoti3(void);
ppm_switch_values_t getPoti4(void);
ppm_switch_values_t getPoti5(void);
ppm_switch_values_t getPoti6(void);
int16_t getPoti7(void);
ppm_switch_values_t getPoti8(void);
void setNick(int16_t value);
void setRoll(int16_t value);
void setGas(int16_t value);
void setYaw(int16_t value);
void setPoti1(ppm_switch_values_t value);
void setPoti2(ppm_switch_values_t value);
void setPoti3(int16_t value);
void setPoti4(ppm_switch_values_t value);
void setPoti5(ppm_switch_values_t value);
void setPoti6(ppm_switch_values_t value);
void setPoti7(int16_t value);
void setPoti8(ppm_switch_values_t value);

#endif /* __STM32F10x_IT_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
