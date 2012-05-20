
#include "sonar.h"
#include "I2C.h"

#define CMD_REAL_RANGING_INCH			0x50
#define CMD_REAL_RANGING_CM				0x51
#define CMD_REAL_RANGING_MS				0x52
#define CMD_FAKE_RANGING_INCH			0x56
#define CMD_FAKE_RANGING_CM				0x57
#define CMD_FAKE_RANGING_MS				0x58

#define ADDR_COMMAND_REGISTER			0x00
#define ADDR_DATA_REGISTER				0x02

#define ADRR_FRONT_SONAR					0xE0

/* Errors */
#define INVALID_SONAR							0xFFFF

#define NUMBER_OF_SONARS					1

#define MEDIAN_FLTER_LENGTH				5

#define OBSTACLE_RANGE						500 //in cm


uint16_t SonarApplyMedianFilter(uint8_t sonar, uint16_t data);


const uint8_t sonar_address_lookup[NUMBER_OF_SONARS] = {ADRR_FRONT_SONAR}; 

static uint16_t median_filter[NUMBER_OF_SONARS][MEDIAN_FLTER_LENGTH] = {100, 100, 100, 100, 100};

bool SonarIsObstacle(uint16_t data) {	
	if (data < OBSTACLE_RANGE) {
		return true;
	} else {
		return false;
	}
}

uint16_t SonarStartRanging(uint8_t sonar) {

	if (sonar >= NUMBER_OF_SONARS) {
		return INVALID_SONAR;
	}

	return I2CSendCommand(sonar_address_lookup[sonar], ADDR_COMMAND_REGISTER, CMD_REAL_RANGING_CM);

}

uint16_t SonarReadData(uint8_t sonar) {

	if (sonar >= NUMBER_OF_SONARS) {
		return INVALID_SONAR;
	}

	return I2CReadData(sonar_address_lookup[sonar], ADDR_DATA_REGISTER);

}	 

uint16_t SonarApplyMedianFilter(uint8_t sonar, uint16_t data) {
	int i,j;

	for (i=0;i<MEDIAN_FLTER_LENGTH;i++) {
		if (data < median_filter[sonar][i]) {
		 	for (j=MEDIAN_FLTER_LENGTH-1;j>i;j++) {
				median_filter[sonar][j] = median_filter[sonar][j-1]; 	
			}

			median_filter[sonar][i] = data;
			break;
		}
	}

	if (i == MEDIAN_FLTER_LENGTH) {
	 	median_filter[sonar][i-1] = data;
	}

	return median_filter[sonar][MEDIAN_FLTER_LENGTH/2];

}
