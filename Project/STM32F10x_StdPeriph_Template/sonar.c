/* ---------------------------------------------------------------------------
** sonar.c
**
** This file provides the interface to communicate with the 
** SRF02 ultrasound sensor (sonar)
**
** Author: Adrian Dudau
** -------------------------------------------------------------------------*/


#include "sonar.h"
#include "I2C.h"

/* IDs of the possible sonar commands to be sent on the I2C bus */
#define CMD_REAL_RANGING_INCH			0x50
#define CMD_REAL_RANGING_CM				0x51
#define CMD_REAL_RANGING_MS				0x52
#define CMD_FAKE_RANGING_INCH			0x56
#define CMD_FAKE_RANGING_CM				0x57
#define CMD_FAKE_RANGING_MS				0x58

/* Addresses of sonar registers for I2C communication */
#define ADDR_COMMAND_REGISTER			0x00
#define ADDR_DATA_REGISTER				0x02

/* I2C addresses for the ultrtasound sensors */
#define ADRR_FRONT_SONAR					0xE0

/* Errors */
#define INVALID_SONAR							0xFFFF

#define NUMBER_OF_SONARS					1

/* Defines for the values returned by the sonar */
#define OBSTACLE_RANGE						500 //in cm
#define NO_OBSTACLE								0

/* ---------------------------------------------------------------------------
** Checks if the read value means an obstacle or not
** -------------------------------------------------------------------------*/
bool SonarIsObstacle(uint16_t data) {	
	if (data != NO_OBSTACLE) {
		return true;
	} else {
		return false;
	}
}

/* ---------------------------------------------------------------------------
** Send the command to the sonar to start the ranging operations
** This operation takes 66 ms before a result is available
** The sonar is kept in busy state while doing the ranging
** -------------------------------------------------------------------------*/
uint16_t SonarStartRanging(uint8_t sonar) {

	if (sonar >= NUMBER_OF_SONARS) {
		return INVALID_SONAR;
	}

	return I2CSendCommand(sonar_address_lookup[sonar], ADDR_COMMAND_REGISTER, CMD_REAL_RANGING_CM);

}

/* ---------------------------------------------------------------------------
** Read the result of a previous ranging operation.
** -------------------------------------------------------------------------*/
uint16_t SonarReadData(uint8_t sonar) {

	if (sonar >= NUMBER_OF_SONARS) {
		return INVALID_SONAR;
	}

	return I2CReadData(sonar_address_lookup[sonar], ADDR_DATA_REGISTER);

}	 
