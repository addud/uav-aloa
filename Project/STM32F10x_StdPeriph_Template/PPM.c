
#include "PPM.h"


//number of PPM channels
#define MAX_CHANNELS	12

//PPPM channels
#define CH_GAS		0
#define CH_ROLL		1
#define CH_NICK		2
#define CH_YAW		3
#define CH_POTI1	4 //SW2	- 2 positions
#define CH_POTI2	5	//CTRL10 - 3 positions
#define CH_POTI3	6	//CTRL6	
#define CH_POTI4 	7	//SW3	- 2 positions
#define CH_POTI5	8	//CTRL9	- 3 positions
#define CH_POTI6	9	//SW8	- 2 positions
#define CH_POTI7	10 //CTRL7 
#define CH_POTI8 	11 //SW9 - 2 positions

//PPM measured pulse period values
#define PPM_MIN_PULSE							(PPM_LOW + PPM_MIN_HIGH) //minimum length of a data pulse
#define PPM_MAX_PULSE							(PPM_LOW + PPM_MAX_HIGH) //maximum length of a data pulse
#define PPM_PERIOD								30000 // total period of the PPM signal
#define PPM_MIN_SYNCH							(PPM_PERIOD - PPM_LOW - MAX_CHANNELS * PPM_MAX_PULSE)	//minimum length of the synch period
#define PPM_MAX_SYNCH							(PPM_PERIOD - PPM_LOW - MAX_CHANNELS * PPM_MIN_PULSE) //maximum length of the synch period
#define PPM_PULSE_HYSTERESIS			200



//PPM pulse level
#define PULSE_HIGH								true
#define PULSE_LOW									false

typedef bool pulse_t;


void RCC_Configuration_PPM(void);
void GPIO_Configuration_PPM(void);
void NVIC_Configuration_PPM(void);
void TIM_Configuration_PPM(void);

bool PPMIsResetWD(void);
bool PPMIsSetWD(void);
void PPMSetWD(void);
void PPMResetWD(void);


TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
EXTI_InitTypeDef   EXTI_InitStructure;
RCC_ClocksTypeDef RCC_ClockFreq;


// The channel array is 0-based!
volatile int16_t PPM_in[MAX_CHANNELS], PPM_diff[MAX_CHANNELS], PPM_out[MAX_CHANNELS] = {0}; //PPM_out[MAX_CHANNELS] =  {-400, -400, 400, 400, 0, 0, 123, -123, -400, -400, 400, 400};
volatile bool PPM_wd = false;

void initPPM(void) {
	   /* System Clocks Configuration */
  RCC_Configuration_PPM();

  /* NVIC configuration */
  NVIC_Configuration_PPM();

  /* Configure the GPIO ports */
  GPIO_Configuration_PPM();

  /* Configure the timers */
  TIM_Configuration_PPM();
}


/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration_PPM(void)
{
 	/* Enable timer clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

}

/**
  * @brief  Configure the GPIOD Pins.
  * @param  None
  * @retval None
  */
void GPIO_Configuration_PPM(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM3 channel 2 pin (PA.07) configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* TIM3 channel 1 pin (PA.06) configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration_PPM(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure ); 

  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

	/* TIM3 configuration --------------------------------------
     The PPM input must be connected TIM3 CH2 pin (PA.07)  
     The PPM ouput must be connected TIM3 CH1 pin (PA.06) 
  ------------------------------------------------------------ */
void TIM_Configuration_PPM(void) {

	/******** Initialize TIM2 **********/

  TIM_DeInit( TIM2 );
	
  TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );

  /* Configuration of timer 2. This timer will generate an
     overflow/update interrupt (TIM2_IRQChannel) every 30ms */
  TIM_TimeBaseStructure.TIM_Period = 29999;
  TIM_TimeBaseStructure.TIM_Prescaler = 71; ////prescale to get 1 tick/us
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );

	TIM_ARRPreloadConfig( TIM2, ENABLE );

	TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE );

	TIM_Cmd( TIM2, ENABLE );
  
	/******** Initialize TIM3 **********/

	TIM_DeInit( TIM3 );

	//config TIM3 timebase
  TIM_TimeBaseStructure.TIM_Period = ( 0xFFFF );
  TIM_TimeBaseStructure.TIM_Prescaler = 71;	//prescale to get 1 tick/us
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit( TIM3, &TIM_TimeBaseStructure );

	/* Input Compare configuration: Channel2 */
	//to capture the PPM signal
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);


	/* Output Compare Toggle Mode configuration: Channel1 */
	//to output the PPM pulses
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1000;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	//disable preloading for the CC register
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 , ENABLE);
  
  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

void TIM2_IRQHandler(void)
{
	PPMSetWD();
	
	// Clear pending-bit of interrupt
  TIM_ClearITPendingBit( TIM2, TIM_IT_Update );

}
/********************************************************************/
/*         Every time a positive edge is detected at PD6            */
/********************************************************************/
/*                               t-Frame
    <----------------------------------------------------------------------->
     ____   ______   _____   ________                ______    sync gap      ____
    |    | |      | |     | |        |              |      |                |
    |    | |      | |     | |        |              |      |                |
 ___|    |_|      |_|     |_|        |_.............|      |________________|
    <-----><-------><------><-----------            <------>                <---
 t0       t1      t2       t4                     tn                     t0

 The PPM-Frame length is 30 ms.
 Channel high pulse width range is 0.6 ms to 1.4 ms completed by an 0.52 ms low pulse.
 The mininimum time delay of two events coding a channel is ( 0.6 + 0.52) ms = 1.12 ms.
 The maximum time delay of two events coding a channel is ( 1.4 + 0.52) ms = 1.92 ms.
 The minimum duration of all channels at minimum value is  12 * 1.12 ms = 13.44 ms.
 The maximum duration of all channels at maximum value is  12 * 1.92 ms = 23.04 ms.
 The remaining time of (30 - 13.44 - 0.52) ms = 16.04 ms  to (30 - 23.04 - 0.52) ms = 6.44 ms is
 the syncronization gap.
 */

//TIM3 interrupt routine
//reads the PPM pulse
void TIM3_IRQHandler(void)
{ 
  static uint8_t index = MAX_CHANNELS, index_in = MAX_CHANNELS, index_out = MAX_CHANNELS;

	static uint16_t oldCapture = 0;

  static uint16_t capture, compare, signal = 0; //in us

	static pulse_t pulse = PULSE_LOW;
	static int16_t elapsedTime = 0;

  if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET) 
  {
    /* Clear TIM3 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

		capture = (uint16_t) TIM_GetCapture2(TIM3);
	
		//if the CCR didn't overflow
		if (capture >= oldCapture)	{
			signal = capture - oldCapture;
		} else {
	 		signal = 0xFFFF - oldCapture + capture;
		}
	  oldCapture = capture;

//		if (index >= MAX_CHANNELS) {
//			index=0;
//		} else {
//			PPM_in[index] = signal-PPM_LOW;
//			index++;
//		}

	  if ((signal > PPM_MIN_SYNCH - PPM_PULSE_HYSTERESIS) && (signal < PPM_MAX_SYNCH + PPM_PULSE_HYSTERESIS)) {
	    index_in = 0;
	  } else { // within the PPM frame
	    if (index_in < MAX_CHANNELS) { // PPM24 supports 12 channels
	      // check for valid signal length
	      if ((signal > PPM_MIN_PULSE - PPM_PULSE_HYSTERESIS) && (signal < PPM_MAX_PULSE + PPM_PULSE_HYSTERESIS)) {
					
						// update channel value
						//value goes between -400 and 400
		        PPM_in[index_in] = signal - PPM_LOW - PPM_NEUTRAL_HIGH; 	// offset of 1.52 ms
						
					index_in++;
	      } 
	    }
	  }
		
		PPMResetWD();

  }	else if(TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET) {
    /* Clear TIM2 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		compare = TIM_GetCapture1(TIM3);

		if (!PPMIsSetWD()) {		

			//if the low period
			if (pulse == PULSE_LOW) {
				compare += PPM_LOW;
				elapsedTime += PPM_LOW; 
				pulse = PULSE_HIGH;
			}	else {
				//if we've forwarded all the channels
			 	if (index_out == MAX_CHANNELS) {
					compare += PPM_PERIOD - elapsedTime;
					elapsedTime = 0;
					index_out = 0;
					 
				} else {
					//if the value must be updated with our value
					if (PPM_out[index_out])	{
						compare += PPM_out[index_out] + PPM_NEUTRAL_HIGH;
						elapsedTime += PPM_out[index_out] + PPM_NEUTRAL_HIGH;
	
						/* Once set, the new PPM difference is applied every time 
						To stop sending it, it must be cleared manually (set to 0) */
	
	//					PPM_out[index_out] = 0;
	
					} else {
						compare += PPM_in[index_out] + PPM_NEUTRAL_HIGH;
						elapsedTime += PPM_in[index_out] + PPM_NEUTRAL_HIGH;
					}
					index_out++;
				}
				pulse = PULSE_LOW;
			}
		} else {
			compare += PPM_PERIOD;	
		}
		TIM_SetCompare1(TIM3, compare);
	}
}
											
bool PPMIsSetWD() {
	return (PPM_wd == true);
}

void PPMSetWD() {
	PPM_wd = true;
}

void PPMResetWD() {
	TIM_SetCounter(TIM2, 0);
	PPM_wd = false;
}

//set a value to a given channel
void setChannel(uint8_t channel, int16_t value) {

//	value += PPM_NEUTRAL_HIGH; 

	if (value < PPM_MIN_HIGH - PPM_NEUTRAL_HIGH) {
		value = PPM_MIN_HIGH;
	} else if (value > PPM_MAX_HIGH  - PPM_NEUTRAL_HIGH) {
		value = PPM_MAX_HIGH;
	}

	PPM_out[channel] = value;
}

ppm_switch_values_t getSwitchValue(int16_t value) {
	if (value >= PPM_MAX_VALUE - PPM_SWITCH_VALUE_JITTER) {
		return SW_ON;
	} else if (value <= PPM_MIN_VALUE + PPM_SWITCH_VALUE_JITTER) {
	 	return SW_OFF;
	}	else if ((value <= PPM_NEUTRAL_VALUE + PPM_SWITCH_VALUE_JITTER) && (value >= PPM_NEUTRAL_VALUE - PPM_SWITCH_VALUE_JITTER)) {
		return SW_NEUTRAL;
	} else {
	 	return SW_UNKNOWN;
	}
} 

void setSwitchValue(uint8_t channel, ppm_switch_values_t value) {
	if (value == SW_ON) {
		setChannel(channel, PPM_MAX_VALUE); 	
	}	else if (value == SW_OFF) {
		setChannel(channel, PPM_MIN_VALUE); 	
	}	else if (value == SW_NEUTRAL) {
		setChannel(channel, PPM_NEUTRAL_VALUE); 	
	}
} 

int16_t getNick() {
	return PPM_in[CH_NICK];
}
int16_t getRoll() {
	return PPM_in[CH_ROLL];
}
int16_t getGas() {
	return PPM_in[CH_GAS];
}
int16_t getYaw() {
	return PPM_in[CH_YAW];
}
ppm_switch_values_t getPoti1() {
	return getSwitchValue(PPM_in[CH_POTI1]);
}
ppm_switch_values_t getPoti2() {
	return getSwitchValue(PPM_in[CH_POTI2]);
}
int16_t getPoti3() {
	return PPM_in[CH_POTI3];
}
ppm_switch_values_t getPoti4() {
	return getSwitchValue(PPM_in[CH_POTI4]);
}
ppm_switch_values_t getPoti5() {
	return getSwitchValue(PPM_in[CH_POTI5]);
}
ppm_switch_values_t getPoti6() {
	return getSwitchValue(PPM_in[CH_POTI6]);
}
int16_t getPoti7() {
	return PPM_in[CH_POTI7];
}
ppm_switch_values_t getPoti8() {
	return getSwitchValue(PPM_in[CH_POTI8]);
}

/* Once set, the new PPM difference is applied every time 
To stop sending it, it must be cleared manually (set to 0) */
void setNick(int16_t value) {
	setChannel(CH_NICK,value);
}
void setRoll(int16_t value) {
	setChannel(CH_ROLL,value);
}
void setGas(int16_t value) {
	setChannel(CH_GAS,value);
}
void setYaw(int16_t value) {
	setChannel(CH_YAW,value);
}
void setPoti1(ppm_switch_values_t value) {
	setSwitchValue(CH_POTI1,value);
}
void setPoti2(ppm_switch_values_t value) {
	setSwitchValue(CH_POTI2,value);
}
void setPoti3(int16_t value) {
	setChannel(CH_POTI3,value);
}
void setPoti4(ppm_switch_values_t value) {
	setSwitchValue(CH_POTI4,value);
}
void setPoti5(ppm_switch_values_t value) {
	setSwitchValue(CH_POTI5,value);
}
void setPoti6(ppm_switch_values_t value) {
	setSwitchValue(CH_POTI6,value);
}
void setPoti7(int16_t value) {
	setChannel(CH_POTI7,value);
}
void setPoti8(ppm_switch_values_t value) {
	setSwitchValue(CH_POTI8,value);
}

