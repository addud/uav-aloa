/* ---------------------------------------------------------------------------
** filter.h
**
** This file provides filtering methods for the values read from the sonar
**
** Author: Adrian Dudau
** -------------------------------------------------------------------------*/

#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f10x.h"


uint16_t MedianFilter(uint16_t datum);

#endif /* __FILTER_H */
