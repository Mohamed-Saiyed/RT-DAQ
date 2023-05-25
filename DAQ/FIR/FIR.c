/***********************************************************
************************************************************
**														  **		
**	FILENAME	:	FIR.c                                 **
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

/*****************************************************************/
/*				    	Include Headers					         */
/*****************************************************************/

#include "main.h"

#include "FIR.h"
#include "FIRConfig.h"

#include "MPU6050.h"

/*****************************************************************/
/*				        Local Macros Definition 		         */
/*****************************************************************/

//

/*****************************************************************/
/*				        Local Types Definition 		             */
/*****************************************************************/

/*
	Type Descreptions : struct to hold each vairable filter operation
	
	SUM_Buffer[AVG_FILTER_ORDER]: Buffer to store the readings of the variable
	SumBufferIndex              : A seprate Index to sum buffer for each variable
*/
typedef struct
{
	FIR_Data_t SUM_Buffer[AVG_FILTER_ORDER];
	uint16_t   SumBufferIndex  	           ;
	
}FilterReadings_s;

typedef struct
{
	FIR_Data_t SUM_Buffer[AVG_FILTER_ORDER_MPU];
	uint16_t   SumBufferIndex  	           ;
	
}MPUFilterReadings_s;

/*****************************************************************/
/*				        Local Variables Definition 		         */
/*****************************************************************/

/*
* Array of sturcts for to store the filter opreations for each variable
*/
FilterReadings_s FilterReadings[AVG_FILTER_NUM_OF_VAR];

/*
* Array of sturcts for to store the filter opreations for each MPU variable
*/
MPUFilterReadings_s MPUFilterReadings[AVG_FILTER_NUM_OF_VAR_MPU];



void Filter_MpuMainFunction(const MPU_Data *DataInput, MPU_Data *DataOutput)
{
	/* variable for the loop index		    				   	 */
	uint16_t LocalIndex   = 0 ;
	/* variable to store the filter average value for each index */
	FIR_Data_t AverageSum[AVG_FILTER_NUM_OF_VAR_MPU] = {0} ;
	
	/* Loop to calclate the average filter value for all IMU sturct variables */
	for(uint8_t StructIndex = 0 , Address = 0 ; StructIndex < AVG_FILTER_NUM_OF_VAR_MPU ; StructIndex++)
	{
		/* Store the variable reading into the filter's buffer */
		MPUFilterReadings[StructIndex].\
		SUM_Buffer[MPUFilterReadings[StructIndex].SumBufferIndex++] = *((float* )((uint32_t)DataInput + Address));
		
		/* check if the sum buffer index reached its maximum */
		if(MPUFilterReadings[StructIndex].SumBufferIndex == AVG_FILTER_ORDER_MPU)
		{
			MPUFilterReadings[StructIndex].SumBufferIndex = 0 ;
		}	
		
		/* Loop to calculate the sum of all the variable readings and store it */
		for(LocalIndex = 0 ; LocalIndex <AVG_FILTER_ORDER_MPU; LocalIndex++ )
		{
			AverageSum[StructIndex] += MPUFilterReadings[StructIndex].SUM_Buffer[LocalIndex];
		}
			
			/* Assign the the average sum variable to the the mpu output sturct */
		  *((float* )((uint32_t)DataOutput + Address)) = AverageSum[StructIndex] / AVG_FILTER_ORDER_MPU;
			
			/* Incerement the address to get the next IMU struct element */		
			Address += 4 ;
	}	
	
	
}


void Filter_MainFunction(FIR_Data_t *DataInput, FIR_Data_t *DataOutput)
{
	/* variable for the loop index								 */
	uint16_t LocalIndex   = 0 ;
	/* variable to store the filter average value for each index */
	FIR_Data_t AverageSum[AVG_FILTER_NUM_OF_VAR] = {0} ;
	
	/* Loop to calclate the average filter value for all input array's index */
	for(uint8_t StructIndex = 0 ; StructIndex < AVG_FILTER_NUM_OF_VAR ; StructIndex++)
	{
		/* Store the variable reading into the filter's buffer */	
		FilterReadings[StructIndex].\
		SUM_Buffer[FilterReadings[StructIndex].SumBufferIndex++] = DataInput[StructIndex];
		
		/* check if the sum buffer index reached its maximum */
		if(FilterReadings[StructIndex].SumBufferIndex == AVG_FILTER_ORDER)
		{
			FilterReadings[StructIndex].SumBufferIndex = 0 ;
		}	
		
		/* Loop to calculate the sum of all the variable readings and store it */
		for(LocalIndex = 0 ; LocalIndex <AVG_FILTER_ORDER; LocalIndex++ )
		{
			AverageSum[StructIndex] += FilterReadings[StructIndex].SUM_Buffer[LocalIndex];
		}
		
		/* Assign the the average sum variable to the the output array */
		DataOutput[StructIndex] = AverageSum[StructIndex] / AVG_FILTER_ORDER;
	}	
	
	
}
