/***********************************************************
************************************************************
**														  **		
**	FILENAME	:	IndProximity.c                        **
**	                                                      **
**	VERSION		: 	1.0.0                                 **
**                                                        **
**	DATE		:	2021-2-1                              **
**                                                        **
**	PLATFORM	:	STM32(STM32F108C8T6)                  **
**                                                        **
**	AUTHOR		:  	MohamedSayed                          **
**                                                        **
**	VENDOR		: 	ASU-RACING-TEAM                       **
**                                                        **
**	                                                      **
**	DESCRIPTION : Inductive Proximity sensor source file  **
**                                                        **
**	MAY BE CHANGED BY USER : No                           **
**                                                        **
***********************************************************/

#include "main.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include "DataLogging.h"

#include "IndProximity.h"
#include "IndProximity_Config.h"

#ifndef PI
#define PI 					3.14159265359
#endif /* PI */

#define TIMER_CHANNEL1_BIT		(1 << 0)
#define TIMER_CHANNEL2_BIT      (1 << 1)
#define TIMER_CHANNEL3_BIT      (1 << 2)
#define TIMER_CHANNEL4_BIT      (1 << 3)



static uint8_t TimerChannelsEventBit[4] = {TIMER_CHANNEL1_BIT , TIMER_CHANNEL2_BIT , TIMER_CHANNEL3_BIT ,TIMER_CHANNEL4_BIT}; 

static EventGroupHandle_t TimerChannelsGroup;

static double ICFreq;
static double ICResolution;

static uint32_t TimerChannelsCaptureValue[8];

static uint32_t TimerChannelsCaptureDiff[4];

static uint16_t WheelRPM[4];
static double   WheelSignalPeriod[4];
static double   WheelSignalFreq[4];

static uint8_t BreakSwitch = 0 ;

static TIM_HandleTypeDef* GlobalConfig = NULL; 

extern DAQStrcut DataAcquisition;

void IndProximity_Init(TIM_HandleTypeDef *htim)
{
	GlobalConfig = htim ;
	
	TimerChannelsGroup = xEventGroupCreate(); 
	
	ICFreq = (HAL_RCC_GetPCLK1Freq() * 2 ) / (GlobalConfig->Init.Prescaler + 1);
	
	ICResolution =  1 / ICFreq;

	
}


void IndProximity_Start(void)
{
	HAL_TIM_IC_Start_IT(GlobalConfig , TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(GlobalConfig , TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(GlobalConfig , TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(GlobalConfig , TIM_CHANNEL_4);
	
}

void IndProximity_GetVehicleSpeed(float* VehicleSpeed)
{
	float FrontWheelsAverageRPM = ((WheelRPM[0] + WheelRPM[1]) / 2);
	float AverageSpeed = ((FrontWheelsAverageRPM / 60 ) * 2 * PI * TIRE_RADIUS) * 3.6;
	//BreakSwitch = !HAL_GPIO_ReadPin(BREAK_SWITCH_PORT , BREAK_SWITCH_PIN);
	//*VehicleSpeed = AverageSpeed / (1 - (AVERAGE_SKID_RATIO * BreakSwitch));
	*VehicleSpeed = AverageSpeed;
}

void IndProximity_GetWheelsRPM(uint16_t* WheelsRPM)
{
	for(uint8_t LocalIndex = 0 ; LocalIndex < 4 ; LocalIndex++)
	{
		WheelsRPM[LocalIndex] = DataAcquisition.WheelsRPM[LocalIndex] ;
	}
}

void IndProximity_MainFunction(void * pvParameters)
{
	TickType_t xLastWakeTime;
	EventBits_t ChannelBitNumber;
	TickType_t xTicksToWait = 2000 / portTICK_PERIOD_MS;
	IndProximity_Start();
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//HAL_GPIO_WritePin(GPIOB , GPIO_PIN_0 , GPIO_PIN_SET);
		ChannelBitNumber = xEventGroupWaitBits(TimerChannelsGroup ,
											   TIMER_CHANNEL1_BIT | TIMER_CHANNEL2_BIT | TIMER_CHANNEL3_BIT | TIMER_CHANNEL4_BIT,
											   pdTRUE,
											   pdFALSE,
											   xTicksToWait);
		if((ChannelBitNumber & 0x0F) != 0x00)
		{
			
			for(uint8_t LocalIndex = 0 ; LocalIndex < 4 ; LocalIndex++)
			{				if((ChannelBitNumber & TimerChannelsEventBit[LocalIndex]) !=0 )
				{
					if(TimerChannelsCaptureValue[(2 * LocalIndex)+1] >  TimerChannelsCaptureValue[2 * LocalIndex])
					{
						TimerChannelsCaptureDiff[LocalIndex] = TimerChannelsCaptureValue[(2 * LocalIndex)+1]\
															   - TimerChannelsCaptureValue[2 * LocalIndex];
					}
					else
					{
						TimerChannelsCaptureDiff[LocalIndex] = (0xFFFF - TimerChannelsCaptureValue[2 * LocalIndex])\
																+ TimerChannelsCaptureValue[(2 * LocalIndex)+1];
					}
						
					WheelSignalPeriod[LocalIndex] =	TimerChannelsCaptureDiff[LocalIndex] * ICResolution;
					
					WheelSignalFreq[LocalIndex]   =	1 / WheelSignalPeriod[LocalIndex] ;
					
					DataAcquisition.WheelsRPM[LocalIndex] = (WheelSignalFreq[LocalIndex] * 60) / PULSE_PER_REV;
	
					TimerChannelsCaptureValue[2 * LocalIndex] =  TimerChannelsCaptureValue[(2 * LocalIndex)+1];
					
				}			
					
			}	
		}
		else
		{
			for(uint8_t LocalIndex = 0 ; LocalIndex < 4 ; LocalIndex++)
			{
				DataAcquisition.WheelsRPM[LocalIndex] = 0 ;
			}
		}
			
				
		//HAL_GPIO_WritePin(GPIOB , GPIO_PIN_0 , GPIO_PIN_RESET);
	   vTaskDelayUntil(&xLastWakeTime , 1);
												
	}
}

BaseType_t  xHigherPriorityTaskWoken = pdFALSE , xResult;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == FRONT_RIGHT_ACTIVE_TIMER_CHANNEL)
	{
		TimerChannelsCaptureValue[1] = __HAL_TIM_GET_COMPARE(htim , FRONT_RIGHT_TIMER_CHANNEL);
		xResult = xEventGroupSetBitsFromISR(TimerChannelsGroup ,TimerChannelsEventBit[0] , 	&xHigherPriorityTaskWoken);
		if( xResult != pdFAIL )
		{
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}		
	}
	if(htim->Channel == FRONT_LEFT_ACTIVE_TIMER_CHANNEL)
	{
		TimerChannelsCaptureValue[3] = __HAL_TIM_GET_COMPARE(htim , FRONT_LEFT_TIMER_CHANNEL);
		xResult = xEventGroupSetBitsFromISR(TimerChannelsGroup ,TimerChannelsEventBit[1] , 	&xHigherPriorityTaskWoken);
		if( xResult != pdFAIL )
		{
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}			
	}
	if(htim->Channel == REAR_RIGHT_ACTIVE_TIMER_CHANNEL)
	{
		TimerChannelsCaptureValue[5] = __HAL_TIM_GET_COMPARE(htim , REAR_RIGHT_TIMER_CHANNEL);
		xResult = xEventGroupSetBitsFromISR(TimerChannelsGroup ,TimerChannelsEventBit[2] , 	&xHigherPriorityTaskWoken); 
		if( xResult != pdFAIL )
		{
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}	
	}
	if(htim->Channel == REAR_LEFT_ACTIVE_TIMER_CHANNEL)
	{
		TimerChannelsCaptureValue[7] = __HAL_TIM_GET_COMPARE(htim , REAR_LEFT_TIMER_CHANNEL);
		xResult = xEventGroupSetBitsFromISR(TimerChannelsGroup ,TimerChannelsEventBit[3] , 	&xHigherPriorityTaskWoken);
		if( xResult != pdFAIL )
		{
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}			
	}

}
