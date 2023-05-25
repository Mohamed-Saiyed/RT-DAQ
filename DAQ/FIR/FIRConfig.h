/***********************************************************
************************************************************
**														  **		
**	FILENAME	:	FIRConfig                             **
**	                                                      **
**	VERSION		: 	1.0.0                                 **
**                                                        **
**	DATE		:	2021-1-5                              **
**                                                        **
**	PLATFORM	:	STM32(STM32F108C8T6)                  **
**                                                        **
**	AUTHOR		:  	MohamedSayed                          **
**                                                        **
**	VENDOR		: 	ASU-RACING-TEAM                       **
**                                                        **
**	                                                      **
**	DESCRIPTION : FIR Average filter Configuration file   **
**                                                        **
**	MAY BE CHANGED BY USER : Yes                          **
**                                                        **
***********************************************************/

#ifndef _MPU_FIR_CONFIG_H
#define _MPU_FIR_CONFIG_H


#define AVG_FILTER_ORDER	      		150 	/* Select the size of the average filter buffer  */
#define AVG_FILTER_NUM_OF_VAR 			6		/* Select the number of vairables to be filtered */



#define AVG_FILTER_ORDER_MPU	        1  
#define AVG_FILTER_NUM_OF_VAR_MPU     6


typedef float FIR_Data_t; /* Slect the type of the filter variable */


#endif /*_MPU_FIR_CONFIG_H*/
