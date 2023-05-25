/***********************************************************
************************************************************
**														  **		
**	FILENAME	:	SUS_Travel.h                          **
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
#ifndef	SUS_TRAVEL_H
#define SUS_TRAVEL_H

#define SUS_CHANNEL_0	(0x00)	3
#define SUS_CHANNEL_1	(0x01)	4
#define SUS_CHANNEL_2	(0x02)	5
#define SUS_CHANNEL_3	(0x03)	6

#define ALL_TRAVEL_SESNORS			(0x10)

#define FRONT_RIGHT_TRAVEL_SENSOR	SUS_CHANNEL_1
#define FRONT_LEFT_TRAVEL_SENSOR	SUS_CHANNEL_0
#define REAR_RIGHT_TRAVEL_SENSOR    SUS_CHANNEL_3
#define REAR_LEFT_TRAVEL_SENSOR     SUS_CHANNEL_2


void SUSTravel_StartDMA(ADC_HandleTypeDef* hadc);
void SUSTravel_MainFunction(void);
void SUSTravel_GetData(uint8_t *TravelReadings , uint8_t WhichSensor);


#endif /*SUS_TRAVEL_H*/

