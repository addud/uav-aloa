#ifndef __SONAR_H
#define __SONAR_H

#include "stm32f10x.h"
#include <stdbool.h>

#define FRONT_SONAR					0

bool SonarIsObstacle(uint16_t data);

uint16_t SonarStartRanging(uint8_t sonar_address);

uint16_t SonarReadData(uint8_t sonar_address);


#endif
