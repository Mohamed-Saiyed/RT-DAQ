/***********************************************************
************************************************************
**														  **		
**	FILENAME	:	IndProximity.h                        **
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

#ifndef	INDPROXIMITY_H
#define INDPROXIMITY_H



void IndProximity_Init(TIM_HandleTypeDef *htim);
void IndProximity_MainFunction(void);
void IndProximity_GetVehicleSpeed(float* VehicleSpeed);
void IndProximity_Start(void);

#endif	/*INDPROXIMITY_H*/
