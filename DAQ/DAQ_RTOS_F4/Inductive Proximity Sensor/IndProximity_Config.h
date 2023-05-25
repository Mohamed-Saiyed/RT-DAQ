/***********************************************************
************************************************************
**														  **		
**	FILENAME	:	IndProximity_Config.h                  **
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
**	DESCRIPTION : Inductive Proximity sensor Config file  **
**                                                        **
**	MAY BE CHANGED BY USER : Yes                          **
**                                                        **
***********************************************************/

#ifndef INDPROXIMITY_CONFIG_H
#define INDPROXIMITY_CONFIG_H

#define FRONT_RIGHT_TIMER_CHANNEL	TIM_CHANNEL_1
#define FRONT_LEFT_TIMER_CHANNEL    TIM_CHANNEL_2
#define REAR_RIGHT_TIMER_CHANNEL    TIM_CHANNEL_3
#define REAR_LEFT_TIMER_CHANNEL     TIM_CHANNEL_4

#define FRONT_RIGHT_ACTIVE_TIMER_CHANNEL	HAL_TIM_ACTIVE_CHANNEL_1
#define FRONT_LEFT_ACTIVE_TIMER_CHANNEL     HAL_TIM_ACTIVE_CHANNEL_2
#define REAR_RIGHT_ACTIVE_TIMER_CHANNEL     HAL_TIM_ACTIVE_CHANNEL_3
#define REAR_LEFT_ACTIVE_TIMER_CHANNEL      HAL_TIM_ACTIVE_CHANNEL_4

#define TIRE_DIAMTER		0.4572	
 
#define TIRE_RADIUS			0.2286
#define PULSE_PER_REV		4
#define AVERAGE_SKID_RATIO	0.2

#define BREAK_SWITCH_PORT	GPIOA
#define BREAK_SWITCH_PIN	GPIO_PIN_7



#endif	/*INDPROXIMITY_CONFIG_H*/

