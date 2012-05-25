/* ---------------------------------------------------------------------------
** PPM.h
**
** This file implements the PPM communication with the radio receiver
** and the FlightCtrl
**
** Author: Adrian Dudau
** -------------------------------------------------------------------------*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PPM_H
#define __PPM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdbool.h>

//PPM measured pulse period values
#define PPM_LOW										520 //low period of each data pulse
#define PPM_MIN_HIGH							600	//minimum high period of a data pulse
#define PPM_NEUTRAL_HIGH					1000 //neutral high period of a data pulse
#define PPM_MAX_HIGH							1400 //maximum high period of a data pulse

//PPM values
#define PPM_MIN_VALUE							(PPM_MIN_HIGH - PPM_NEUTRAL_HIGH)
#define PPM_MAX_VALUE							(PPM_MAX_HIGH - PPM_NEUTRAL_HIGH)
#define PPM_NEUTRAL_VALUE					0
#define PPM_SWITCH_VALUE_JITTER		50
		 
//type of the possible states of a 3-way switch
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

#endif /* __PPM_H */
