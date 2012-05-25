/* ---------------------------------------------------------------------------
** main.c
**
** This file provides the main functionality of the application.
** It initializes the application modules and implements the concurrent tasks
** implementing the Obstacle Avoidance (OA) and Automated Landing	(AL) features
**
** Authors: Adrian Dudau, Adrian Caragea
** -------------------------------------------------------------------------*/

#include "stm32f10x.h"
#include "stm32_eval.h"
#include <stdio.h>
#include "PPM.h"
#include "I2C.h"
#include "sonar.h"
#include "filter.h"


#ifdef USE_STM32100B_EVAL
 #include "stm32100b_eval_lcd.h"
#elif defined USE_STM3210B_EVAL
 #include "stm3210b_eval_lcd.h"
#elif defined USE_STM3210E_EVAL
 #include "stm3210e_eval_lcd.h" 
#elif defined USE_STM3210C_EVAL
 #include "stm3210c_eval_lcd.h"
#elif defined USE_STM32100E_EVAL
 #include "stm32100e_eval_lcd.h"
#endif

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x_it.h"

/* Defines for the AL module. */
#define OUT_OF_RANGE        160//0xFFFF
#define Kp                  0.5
#define Kd                  3
#define Tsample             (50 / portTICK_RATE_MS)

/* Defines for the OA module. */
#define OA_NICK_GAIN					50
#define OA_GAS_GAIN						80
#define OA_ROLL_GAIN					50

/* The number of cycles the kopter keeps avoiding the obstacle after 
** the obstacle is not detected anymor.
** This helps compensate for the limited angle view of the front sonar.
*/ 
#define OA_INERTIAL_TIMEOUT		5


/* Some variables for the AL module. */
volatile uint16_t readVoltage = 0;
volatile uint16_t distanceCm = 0;
volatile uint16_t distanceAverage = 0;
volatile int16_t distanceCurrent = 0;
volatile int16_t distanceOld = 0;
volatile int16_t throttleDiff = 0;

uint16_t landingLimit = 35;

int16_t altitudeHold = 0, positionHold = 0;
int16_t autoLanding = 0;

/*
******** Connections for the no-name chinese devboard *************
** SCL -> PB6 -> XS2-10
** SDA -> PB7 -> XS2-4
**
** PPM_Receiver -> PA7 -> XS2-6
** PPM_FlightCtrl -> PA6 -> XS2-8
*/

void GPIO_Configuration_ADC(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure PC.04 (ADC Channel14) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void RCC_Configuration_ADC(void)
{
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
  /* ADCCLK = PCLK2/2 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
#else
  /* ADCCLK = PCLK2/4 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
#endif
  /* Enable peripheral clocks ------------------------------------------------*/

  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
}


/* ---------------------------------------------------------------------------
** Implements the OA feature
** 
** Basic functionality: 
** When the user enables the OA feature, the kopter simulates going to a waypoint
** by going forwards. Once an obstacle is detected in front, it stops going
** forward and tries to avoid it through the left. Once the obstacle has been 
** cleared, the kopter resumes going forward. (simulates continuing towards the waypoint)
**
** -------------------------------------------------------------------------*/
void OATask(void *pvParameters)
{
	static uint16_t data;
	static uint8_t inertial_timeout = 0;
  portTickType lastWake = xTaskGetTickCount();

	I2CInit();

	for(;;) {
		if (getPoti5() == SW_ON) {
			
			// Start the sonar ranging
			SonarStartRanging(FRONT_SONAR);
			
			//wait until raning is done
	    vTaskDelay(SONAR_RESPONSE_DELAY / portTICK_RATE_MS);

			data = SonarReadData(FRONT_SONAR);

			//check for communication errors 
			if (IS_I2C_ERROR(data)) {
				//  return control of the	kopter to the user
				setNick(PPM_NEUTRAL_VALUE);	
				setRoll(PPM_NEUTRAL_VALUE);
//				setGas(PPM_NEUTRAL_VALUE);
			
			} else {
				//filter the data
				data = MedianFilter(data);
	
				// The user can flip the Poti8 switch to force the kopter to go backwards while in OA mode
				// helps to quickly avoid crashes in case something goes wrong
				if (getPoti8() ==  SW_ON) {
	
//					inertial_timeout = OA_INERTIAL_TIMEOUT;
					setNick(getNick() - OA_NICK_GAIN);	
					
				}

				if (SonarIsObstacle(data)) {
	
					inertial_timeout = OA_INERTIAL_TIMEOUT;
					
				} 
				
				if (inertial_timeout > 0) {
					//if we still need to keep on avoiding the obstacle
					// don't go forwards anymore - the user still can go forwards manually if needed
					setNick(PPM_NEUTRAL_VALUE);
//					setGas(getGas() + OA_GAS_GAIN);
					//go left
					setRoll(getRoll() + OA_ROLL_GAIN);
								
					inertial_timeout--;
	
				} else {
					//resume going forwards
					setNick(getNick() + OA_NICK_GAIN);
					// don't go left anymore - the user still can go left manually if needed
					setRoll(PPM_NEUTRAL_VALUE);
//					setGas(PPM_NEUTRAL_VALUE);
	
				}
	
			}

		} else {
			//if OA function is not enabled, return control to the user
			setNick(PPM_NEUTRAL_VALUE);
			setRoll(PPM_NEUTRAL_VALUE);
			//run the task with a 10 ms period while the OA function is not enabled
			//monitors the flip of the OA switch
			vTaskDelayUntil(&lastWake, 10 / portTICK_RATE_MS);
		}
  }
  
}


void DataTask(void *pvParameters)
{
  ADC_InitTypeDef ADC_InitStructure;
  portTickType lastWake = xTaskGetTickCount();
  uint8_t i = 0;
  uint16_t distances[4];
  static uint8_t calibrationFlag = true;
  

  // init ADC
  /* System clocks configuration ---------------------------------------------*/
  RCC_Configuration_ADC();

  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration_ADC();
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel14 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_55Cycles5);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  /* --------------------------------------------------------------*/
  /*----------------- Landing Limit Calibratin --------------------*/
                  
  while(calibrationFlag == true)
  {
    readVoltage = (uint16_t)((uint32_t)(3000 * ADC_GetConversionValue(ADC1)) / 0x1000); // conversion to voltage "mV" units
    //liniarize the IR sensor curve equation
    if((readVoltage > 850) && (readVoltage < 2150)){
      //distanceCm = (uint16_t)((uint32_t)(3460 - readVoltage) / 38);
      distanceCm = (uint16_t)((uint32_t)(3*(2575 - readVoltage)) / 86);
    }
    else if((readVoltage > 400) && (readVoltage <= 850)){
      //distanceCm = (uint16_t)((uint32_t)(3 * (1400 - readVoltage)) / 20);
      distanceCm = (uint16_t)((1150 - readVoltage) / 5);
    }
    else{
      distanceCm = OUT_OF_RANGE;  // out of range
    }

    //store last measurement
    if( distanceCm != OUT_OF_RANGE ){
      distances[i] =  distanceCm;
      i++;
    }
    else{
      for(i = 0; i < 4; i++) distances[i] = OUT_OF_RANGE;
      i = 0;
    }
    
    if(i > 3) i = 0;  //ring buffer

    // average of the last 4 measurements
    if(distances[3] != OUT_OF_RANGE){
      distanceAverage = (uint16_t)((uint32_t)(distances[0] + distances[1] + distances[2] + distances[3]) / 4);
      
      if(calibrationFlag == true)
      {
        landingLimit = distanceAverage + 5; // landing limit calibration
        calibrationFlag = false; 
      } 
    }
    else{
      distanceAverage = OUT_OF_RANGE;
    }
  }

  /*--------------------------------------------------------------------------------*/


  for(;;) {

    readVoltage = (uint16_t)((uint32_t)(3000 * ADC_GetConversionValue(ADC1)) / 0x1000); // conversion to voltage "mV" units
    //liniarize the IR sensor curve equation
    if((readVoltage > 850) && (readVoltage < 2150)){
      //distanceCm = (uint16_t)((uint32_t)(3460 - readVoltage) / 38);
      distanceCm = (uint16_t)((uint32_t)(3*(2575 - readVoltage)) / 86);
    }
    else if((readVoltage > 400) && (readVoltage <= 850)){
      //distanceCm = (uint16_t)((uint32_t)(3 * (1400 - readVoltage)) / 20);
      distanceCm = (uint16_t)((1150 - readVoltage) / 5);
    }
    else{
      distanceCm = OUT_OF_RANGE;  // out of range
    }

    //store last measurement
    if( distanceCm != OUT_OF_RANGE ){
      distances[i] =  distanceCm;
      i++;
    }
    else{
      for(i = 0; i < 4; i++) distances[i] = OUT_OF_RANGE;
      i = 0;
    }
    
    if(i > 3) i = 0;  //ring buffer

    // average of the last 4 measurements
    if(distances[3] != OUT_OF_RANGE){
      distanceAverage = (uint16_t)((uint32_t)(distances[0] + distances[1] + distances[2] + distances[3]) / 4);
    }
    else{
      distanceAverage = OUT_OF_RANGE;
    }

    vTaskDelayUntil(&lastWake, 50 / portTICK_RATE_MS);
  }
}


void PlannerTask(void *pvParameters){


  portTickType lastWake = xTaskGetTickCount();
static int16_t distanceP, distanceD;
static int16_t throttlePPM;

  for(;;){

    // check hovering flag(altitude hold) && autolanding switch
    // read distance
    // read hovering Throttle value
    // PID decrese hovering value and measure the altitude
    // loop till "0 m"
  
    altitudeHold = getPoti1();
    positionHold = getPoti2();
    autoLanding = getPoti6();

    if((altitudeHold == SW_ON) && (positionHold == SW_NEUTRAL) && (autoLanding == SW_ON))
    {
      throttlePPM = getGas();
  
      distanceCurrent = (int16_t)distanceAverage; //use mutex
  
      distanceP = distanceCurrent - 0;
      distanceD = distanceOld - distanceCurrent;
  
      throttleDiff = (Kp * distanceP) - (Kd * distanceD);// / Tsample);
  
      throttlePPM -= (int16_t)throttleDiff;
  
      // set new throttle value
      setGas(throttlePPM);
      //setGas((-1)*(int16_t)throttleDiff);
  
      distanceOld = distanceCurrent;

      if(distanceCurrent < landingLimit)
      {
        setGas(PPM_MIN_VALUE);
        setYaw(PPM_MAX_VALUE);
      }
    }
    else
    {
//			if (getPoti5() != SW_ON) {
				setGas(PPM_NEUTRAL_VALUE);
//			}
      setYaw(PPM_NEUTRAL_VALUE);
    }        

    vTaskDelayUntil(&lastWake, 50 / portTICK_RATE_MS);
  }
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{

  /*With ST you have to ensure the interrupt priority bits are all set to 
  preemption priority, rather than sub priority, to make your life easier. 
  ST seem to stand out from the crowd by not doing that by default. Using 
  the ST library, it is normally done using */
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	initPPM();

  xTaskCreate( DataTask, ( signed portCHAR * ) "DataTask", configMINIMAL_STACK_SIZE+64, (void*)NULL, 4, NULL );
  xTaskCreate( PlannerTask, ( signed portCHAR * ) "PlannerTask", configMINIMAL_STACK_SIZE+128, (void*)NULL, 3, NULL );
  xTaskCreate( OATask, ( signed portCHAR * ) "ObstacleAvoidanceTask", configMINIMAL_STACK_SIZE, (void*)NULL, 5, NULL );
  
  /* Start the scheduler. */
  vTaskStartScheduler();
     

  /* Will only get here if there was not enough heap space to create the
  idle task. */
  return 0;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
