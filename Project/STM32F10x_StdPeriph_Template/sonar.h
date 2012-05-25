/* ---------------------------------------------------------------------------
** sonar.h
**
** This file provides the interface to communicate with the 
** ultrasound sensor (sonar)
**
** Author: Adrian Dudau
** -------------------------------------------------------------------------*/

#ifndef __SONAR_H
#define __SONAR_H

#include "stm32f10x.h"
#include <stdbool.h>

/* Indexes of the sonars */ 
#define FRONT_SONAR							0

#define SONAR_RESPONSE_DELAY		70

bool SonarIsObstacle(uint16_t data);

uint16_t SonarStartRanging(uint8_t sonar_address);

uint16_t SonarReadData(uint8_t sonar_address);


#endif /* __SONAR_H */
