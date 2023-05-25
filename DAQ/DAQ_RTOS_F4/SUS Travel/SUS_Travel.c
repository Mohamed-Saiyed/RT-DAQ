/***********************************************************
************************************************************
**														  **		
**	FILENAME	:	SUS_Travel.c                          **
**	                                                      **
**	VERSION		: 	1.0.0                                 **
**                                                        **
**	DATE		:	2021-2-3                              **
**                                                        **
**	PLATFORM	:	STM32(STM32F108C8T6)                  **
**                                                        **
**	AUTHOR		:  	MohamedSayed                          **
**                                                        **
**	VENDOR		: 	ASU-RACING-TEAM                       **
**                                                        **
**	                                                      **
**	DESCRIPTION : Travel Sensors Source File              **
**                                                        **
**	MAY BE CHANGED BY USER : NO                           **
**                                                        **
***********************************************************/	

#include "main.h"

#include "DataLogging.h"

#include "SUS_Travel.h"

#define MAX_ADC_READ		4095

#define	LINEAR_POT_PEAK		100



volatile uint32_t ADC_val[4];


extern DAQStrcut DataAcquisition;

void SUSTravel_StartDMA(ADC_HandleTypeDef* hadc)
{
	HAL_ADC_Start_DMA(hadc,(uint32_t *)ADC_val,4);
}

void SUSTravel_MainFunction(void)
{
	for(uint8_t LocalIndex = 0 ; LocalIndex < 4 ; LocalIndex++)
	{
		DataAcquisition.WheelsTravel[LocalIndex]  =  (uint8_t)((ADC_val[LocalIndex] * LINEAR_POT_PEAK) / MAX_ADC_READ);
	}
}

void SUSTravel_GetData(uint8_t *TravelReadings , uint8_t WhichSensor)
{
	if(WhichSensor == ALL_TRAVEL_SESNORS)
	{
		for(uint8_t LocalIndex = 0 ; LocalIndex < 4 ; LocalIndex++)
		{		
			//DataAcquisition.WheelsTravel[LocalIndex] = SUSTravelReadings[LocalIndex];
		}
	}
	else
	{
//		if((FRONT_RIGHT_TRAVEL_SENSOR & WhichSensor) == FRONT_RIGHT_TRAVEL_SENSOR)
//		{
//			TravelReadings[FRONT_RIGHT_TRAVEL_SENSOR / 2] = SUSTravelReadings[FRONT_RIGHT_TRAVEL_SENSOR / 2];
//		}
//		if((FRONT_LEFT_TRAVEL_SENSOR & WhichSensor) == FRONT_LEFT_TRAVEL_SENSOR)
//		{
//			TravelReadings[FRONT_RIGHT_TRAVEL_SENSOR / 2] = SUSTravelReadings[FRONT_RIGHT_TRAVEL_SENSOR / 2];
//		}
//		if((REAR_RIGHT_TRAVEL_SENSOR & WhichSensor) == REAR_RIGHT_TRAVEL_SENSOR)
//		{
//			TravelReadings[FRONT_RIGHT_TRAVEL_SENSOR / 2] = SUSTravelReadings[FRONT_RIGHT_TRAVEL_SENSOR / 2];
//		}
//		if((REAR_LEFT_TRAVEL_SENSOR & WhichSensor) == REAR_LEFT_TRAVEL_SENSOR)
//		{
//			TravelReadings[REAR_LEFT_TRAVEL_SENSOR / 2] = SUSTravelReadings[REAR_LEFT_TRAVEL_SENSOR / 2];
//		}
	}
	
}





