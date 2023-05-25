/***********************************************************
************************************************************
**														  **		
**	FILENAME	:	FIR.h                                 **
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
**	DESCRIPTION : FIR Average filter source file          **
**                                                        **
**	MAY BE CHANGED BY USER : No                           **
**                                                        **
***********************************************************/


#ifndef _MPU_FIR_H
#define _MPU_FIR_H

/*****************************************************************/
/*				    	Include Headers					         */
/*****************************************************************/

#include "FIRConfig.h"

#include "MPU6050.h"


/*****************************************************************/
/*				        Macros Definition       		         */
/*****************************************************************/

//

/*****************************************************************/
/*				        Types Definition        		         */
/*****************************************************************/

//


/*****************************************************************/
/*				        Functions Prototype        		         */
/*****************************************************************/

void Filter_MainFunction(FIR_Data_t *DataInput, FIR_Data_t *DataOutput);
void Filter_MpuMainFunction(const MPU_Data *DataInput, MPU_Data *DataOutput);



#endif /*_MPU_FIR_H*/

