#include "main.h"

#include "DataLogging.h"
#include "Encoder.h"

/*max angle before the timer auto reset*/
#define MAX_ANGLE 180


/*Number of pulses per one revolution "dependes on the encoder"*/
#define PPR  360 

volatile int16_t steering_wheel_angle = 0;

static TIM_HandleTypeDef* GlobalConfig = NULL;


extern DAQStrcut DataAcquisition;


void Encoder_Setupt(TIM_HandleTypeDef *htim)
{
	if(htim != NULL)
	{
		GlobalConfig = htim;
	}
	else
	{
		
	}
	
}

void Encoder_Start(void)
{
	HAL_TIM_Encoder_Start(GlobalConfig, TIM_CHANNEL_1|TIM_CHANNEL_2);
}

void TIM_ResetCounter(TIM_TypeDef* TIMx)
{
	
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
	
	/*setting the counter register to zero to reset*/
  TIMx->CNT = 0; //to be tested
}

void Encoder_MainFuction(void)
{
	int16_t counter = 0 ,  position = 0;
	/*type casting the data so that 65534 >> -1*/
	counter =(int16_t) __HAL_TIM_GET_COUNTER(GlobalConfig);
	
	/*calculating the exact angle by knowing how many pulses per revolution*/
	position = (counter)*360/PPR;
	
	/*checking if the angle is a possible value and reseting it if otherwise*/
	if ( (position >= MAX_ANGLE) || (position <= (-1*MAX_ANGLE)))
		{
			TIM_ResetCounter(TIM2);
			position =0 ;
		}
	 if(position != 127)
	 {
	 
		DataAcquisition.SteeringAngle = (position);
	 }
}

void Encoder_GetSteeringAngle(int16_t* Steering_Angle)
{
	if(Steering_Angle != NULL)
	{
		*Steering_Angle = steering_wheel_angle;
	}
	else
	{
		
	}
	
}