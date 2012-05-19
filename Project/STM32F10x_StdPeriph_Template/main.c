/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_eval.h"
#include <stdio.h>
#include "PPM.h"

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

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef USE_STM32100B_EVAL
  #define MESSAGE1   "STM32 MD Value Line " 
  #define MESSAGE2   " Device running on  " 
  #define MESSAGE3   "  STM32100B-EVAL    " 
#elif defined (USE_STM3210B_EVAL)
  #define MESSAGE1   "STM32 Medium Density" 
  #define MESSAGE2   " Device running on  " 
  #define MESSAGE3   "   STM3210B-EVAL    " 
#elif defined (STM32F10X_XL) && defined (USE_STM3210E_EVAL)
  #define MESSAGE1   "  STM32 XL Density  " 
  #define MESSAGE2   " Device running on  " 
  #define MESSAGE3   "   STM3210E-EVAL    "
#elif defined (USE_STM3210E_EVAL)
  #define MESSAGE1   " STM32 High Density " 
  #define MESSAGE2   " Device running on  " 
  #define MESSAGE3   "   STM3210E-EVAL    " 
#elif defined (USE_STM3210C_EVAL)
  #define MESSAGE1   " STM32 Connectivity " 
  #define MESSAGE2   " Line Device running" 
  #define MESSAGE3   " on STM3210C-EVAL   "
#elif defined (USE_STM32100E_EVAL)
  #define MESSAGE1   "STM32 HD Value Line " 
  #define MESSAGE2   " Device running on  " 
  #define MESSAGE3   "  STM32100E-EVAL    "   
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
 USART_InitTypeDef USART_InitStructure;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


#define OUT_OF_RANGE        0xFFFF
#define Kp                  0.034
#define Kd                  0
#define Tsample             (50 / portTICK_RATE_MS)

volatile uint16_t readVoltage = 0;
volatile uint16_t distanceCm = 0;
volatile uint16_t distanceAverage = 0;
volatile uint16_t distanceCurrent = 0;
volatile uint16_t distanceOld = 0;
volatile uint16_t throttleDiff = 0;

int16_t altitudeHold = 0, positionHold = 0;
int16_t autoLanding = 0;

/* Private functions ---------------------------------------------------------*/
void MyTask(void *pvParameters)
{
	int MyCnt=0;

	for(;;) {
		MyCnt++;
    vTaskDelay(100 / portTICK_RATE_MS);
  }

}

void AddTask(void *pvParameters)
{
	int AddCnt=0;

	for(;;) {
		AddCnt++;
    vTaskDelay(100 / portTICK_RATE_MS);
  }

}

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

void DataTask(void *pvParameters)
{
 ADC_InitTypeDef ADC_InitStructure;
  portTickType lastWake = xTaskGetTickCount();
  uint8_t i = 0;
  uint16_t distances[4];
  

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




  for(;;) {

    readVoltage = (uint16_t)((uint32_t)(3000 * ADC_GetConversionValue(ADC1)) / 0xFFF); // conversion to voltage "mV" units
    //liniarize the IR sensor curve equation
    if((readVoltage > 1000) && (readVoltage < 2700)){
      distanceCm = (uint16_t)((uint32_t)(3460 - readVoltage) / 38);
    }
    else if((readVoltage > 400) && (readVoltage <= 1000)){
      distanceCm = (uint16_t)((uint32_t)(3 * (1040 - readVoltage)) / 20);
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
  uint16_t distanceP, distanceD;
  int16_t throttlePPM;

  for(;;){

    // check hovering flag(altitude hold) && autolanding switch
    // read distance
    // read hovering Throttle value
    // PID decrese hovering value and measure the altitude
    // loop till "0 m"
  
    //altitudeHold = getPoti1();
    //positionHold = getPoti2();
    //autoLanding = getPoti6();

    if((altitudeHold > 0x390) && ((positionHold > -20) && (positionHold < 20)) && (autoLanding > 390))
    {
      throttlePPM = getGas();
  
      distanceCurrent = distanceAverage; //use mutex
  
      distanceP = distanceCurrent - 0;
      distanceD = distanceOld - distanceCurrent;
  
      throttleDiff = Kp * distanceP + Kd * distanceD / Tsample;
  
      throttlePPM -= throttleDiff;
  
      // set new throttle value
      setGas(throttlePPM);
  
      distanceOld = distanceCurrent;
    }        

    vTaskDelayUntil(&lastWake, 100 / portTICK_RATE_MS);
  }
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

//
//  /* USARTx configured as follow:
//        - BaudRate = 115200 baud  
//        - Word Length = 8 Bits
//        - One Stop Bit
//        - No parity
//        - Hardware flow control disabled (RTS and CTS signals)
//        - Receive and transmit enabled
//  */
//  USART_InitStructure.USART_BaudRate = 115200;
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;
//  USART_InitStructure.USART_Parity = USART_Parity_No;
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//
//  STM_EVAL_COMInit(COM1, &USART_InitStructure);


  /* Add your application code here */


  /*With ST you have to ensure the interrupt priority bits are all set to 
  preemption priority, rather than sub priority, to make your life easier. 
  ST seem to stand out from the crowd by not doing that by default. Using 
  the ST library, it is normally done using */
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	initPPM();

  //xTaskCreate( MyTask, ( signed portCHAR * ) "MyTask", configMINIMAL_STACK_SIZE, (void*)NULL, 2, NULL );
  //xTaskCreate( AddTask, ( signed portCHAR * ) "AddTask", configMINIMAL_STACK_SIZE, (void*)NULL, 2, NULL );

  xTaskCreate( DataTask, ( signed portCHAR * ) "DataTask", configMINIMAL_STACK_SIZE+64, (void*)NULL, 2, NULL );
  xTaskCreate( PlannerTask, ( signed portCHAR * ) "PlannerTask", configMINIMAL_STACK_SIZE+128, (void*)NULL, 3, NULL );
  
  
  /* Start the scheduler. */
  vTaskStartScheduler();
     

  /* Will only get here if there was not enough heap space to create the
  idle task. */
  return 0;
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  //USART_SendData(EVAL_COM1, (uint8_t) ch);

  /* Loop until the end of transmission */
  //while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
  {}

  return ch;
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
