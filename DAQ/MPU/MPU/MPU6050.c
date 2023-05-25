/***********************************************************
************************************************************
**														  **		
**	FILENAME	:	MPU6050.c                             **
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
**	DESCRIPTION : MPU6050(IMU) Module source file         **
**                                                        **
**	MAY BE CHANGED BY USER : No                           **
**                                                        **
***********************************************************/

/****************************************************************************************/

/*****************************************************************/
/*				    	Include Headers					         */
/*****************************************************************/
#include "MPU6050.h"


#include "main.h"

/*****************************************************************/
/*				        Local Macros Definition 		         */
/*****************************************************************/

#define MPU_IS_PRESENT 		104

/*****************************************************************/
/*				        Local Types Definition 		             */
/*****************************************************************/


/*****************************************************************/
/*				        Local Variables Definition 		         */
/*****************************************************************/

/*variable to save the Accelrometer Divider         */
static float AccelDivider = 0 ;
/*variable to save the Gyroscope Divider            */
static float GyroDivider  = 0 ;		

/*variable to save MPU6050 Initialzations state     */
static uint8_t MpuModuleState = 0 ;

/*pointer to save sensor I2c configrations prameters*/
static I2C_HandleTypeDef* GlobalConfig = NULL;

/****************************************************************************************/

void MPU_Init(I2C_HandleTypeDef *hi2c , IMU_AccelScaleRange AccelScale, IMU_GyroScaleRange GyroScale)
{
	/*Check if the moudle is initialzed*/
	if( MpuModuleState == 1 )
	{
		/*Do Nothing*/
	}
	/*Check for a null pointer         */
	else if(hi2c == NULL)
	{
		/*Do nothing and Don't contunie*/
	}
	else
	{
		/*Variable to save the return value of IMU'S WHO_AM_I REG                    */
		uint8_t Check = 0 ;
		/*Variable to the configuraions of the IMU to be passed on the write function*/
		uint8_t Data  = 0 ;

		/*Save the configurations pointer to be used in other APIs                    */
		GlobalConfig = (I2C_HandleTypeDef*)hi2c ;
		
		/*Set the Accelrameter divider*/
		switch(AccelScale)
		{
			case IMU_ACCEL_SCALE_2g	: AccelDivider = MPU6050_ACCE_DIVIDER_2	; break;
			case IMU_ACCEL_SCALE_4g	: AccelDivider = MPU6050_ACCE_DIVIDER_4	; break;
			case IMU_ACCEL_SCALE_8g : AccelDivider = MPU6050_ACCE_DIVIDER_8	; break;
			case IMU_ACCEL_SCALE_16g: AccelDivider = MPU6050_ACCE_DIVIDER_16; break;
			default: AccelDivider = MPU6050_ACCE_DIVIDER_2; break;
		}
		/*Set the Gyroscope divider*/
		switch(GyroScale)
		{
			case IMU_GYRO_SCALE_250	: GyroDivider = MPU6050_GYRO_DIVIDER_250 ; break;
			case IMU_GYRO_SCALE_500	: GyroDivider = MPU6050_GYRO_DIVIDER_500 ; break;
			case IMU_GYRO_SCALE_1000: GyroDivider = MPU6050_GYRO_DIVIDER_1000; break;
			case IMU_GYRO_SCALE_2000: GyroDivider = MPU6050_GYRO_DIVIDER_2000; break;
			default: GyroDivider = MPU6050_GYRO_DIVIDER_250; break;
		}
		
		/*check for the IMU module presence*/
		HAL_I2C_Mem_Read(GlobalConfig , MPU6050_SLAVE_ADDR , WHO_AM_I , 1 , &Check , 1 , HAL_MAX_DELAY);

		/*continue on doing IMU's configs if the moudule is Present*/
		if(Check == MPU_IS_PRESENT)
		{
			/*Wake up the MPU sensor up       */
			Data = 1 ;		
			HAL_I2C_Mem_Write(GlobalConfig , MPU6050_SLAVE_ADDR , PWR_MGMT_1 , 1 , &Data , 1 , HAL_MAX_DELAY);
		
			/*Set the sample rate to 1KHz     */
			Data = 0x07;
			HAL_I2C_Mem_Write(GlobalConfig , MPU6050_SLAVE_ADDR , SMPLRT_DIV , 1 , &Data , 1 , HAL_MAX_DELAY);
		
			/*Set the Accelrometer scale range*/
			Data = AccelScale;
			HAL_I2C_Mem_Write(GlobalConfig , MPU6050_SLAVE_ADDR , ACCEL_CONFIG , 1 , &Data , 1 , HAL_MAX_DELAY);
		
			/*Set the Gyroscope scale range   */
			Data = GyroScale;
			HAL_I2C_Mem_Write(GlobalConfig , MPU6050_SLAVE_ADDR , GYRO_CONFIG , 1 , &Data , 1 , HAL_MAX_DELAY);
			
			/*Set the module state to be ready*/
			MpuModuleState = 1 ;
		}

	}

}

void MPU_AccelRead(MPU_Data *AccelData)
{
	if( MpuModuleState == 0 )
	{
		/*Do Nothing if the sensor is not initialzed*/
	}
	else
	{
		/*Local array to save the data read from MPU6050*/
		uint8_t MPU_Date[6] ;
		/*variables to save the Accelrometer data       */
		int16_t ACCEL_X ;
		int16_t ACCEL_Y ;
		int16_t ACCEL_Z ;
		
		/*Read the Accelrometer data from MPU6050*/
		HAL_I2C_Mem_Read(GlobalConfig , MPU6050_SLAVE_ADDR , ACCEL_XOUT_H , 1 , MPU_Date , 6 , HAL_MAX_DELAY);
		
		/*Rearrange the mpu6050 Accelrometer data*/
		ACCEL_X = (int16_t)(MPU_Date[0] << 8 | MPU_Date[1]);
		ACCEL_Y = (int16_t)(MPU_Date[2] << 8 | MPU_Date[3]);
		ACCEL_Z = (int16_t)(MPU_Date[4] << 8 | MPU_Date[5]);
		
		/*Save mpu6050's Accelrameter data into the user's struct*/
		AccelData->Ax = ACCEL_X / AccelDivider ;
		AccelData->Ay = ACCEL_Y / AccelDivider ;
		AccelData->Az = ACCEL_Z / AccelDivider ;
		
		
	}
	
	

}

void MPU_GyroRead(MPU_Data *GyroData)
{
	if( MpuModuleState == 0 )
	{
		/*Do Nothing if the sensor is not initialzed*/
	}
	else
	{
		/*Local array to save the data read from MPU6050*/
		uint8_t MPU_Date[6] ;
		/*variables to save the Gyroscope data          */
		int16_t Gyro_X ;
		int16_t Gyro_Y ;
		int16_t Gyro_Z ;
		
		/*Read the Gyroscope data from MPU6050*/
		HAL_I2C_Mem_Read(GlobalConfig , MPU6050_SLAVE_ADDR , GYRO_XOUT_H , 1 , MPU_Date , 6 , HAL_MAX_DELAY);
		
		/*Rearrange the mpu6050 Gyroscope data*/
		Gyro_X = (int16_t)(MPU_Date[0] << 8 | MPU_Date[1]);
		Gyro_Y = (int16_t)(MPU_Date[2] << 8 | MPU_Date[3]);
		Gyro_Z = (int16_t)(MPU_Date[4] << 8 | MPU_Date[5]);
		
		/*Save mpu6050's Gyroscope data into the user's struct*/
		GyroData->Gx = Gyro_X / GyroDivider ;
		GyroData->Gy = Gyro_Y / GyroDivider ;
		GyroData->Gz = Gyro_Z / GyroDivider ;
	}

}

void MPU_ReadAll(MPU_Data *Data)
{
	if( MpuModuleState == 0 )
	{
		/*Do Nothing if the sensor is not initialzed*/
	}
	else
	{
		/*Local array to save the data read from MPU6050*/
		uint8_t MPU_Date[14] ;
		
		/*variables to save All IMU's data              */
		int16_t ACCEL_X ;
		int16_t ACCEL_Y ;
		int16_t ACCEL_Z ;
		int16_t Temp_Raw;
		int16_t Gyro_X  ;
		int16_t Gyro_Y  ;
		int16_t Gyro_Z  ;
		
		/*Read all MPU6050's Data*/
		HAL_I2C_Mem_Read(GlobalConfig , MPU6050_SLAVE_ADDR , ACCEL_XOUT_H , 1 , MPU_Date , 14 , HAL_MAX_DELAY);
		
		/*Rearrange the mpu6050' data*/
		ACCEL_X = (int16_t)(MPU_Date[0]  << 8 | MPU_Date[1] );
		ACCEL_Y = (int16_t)(MPU_Date[2]  << 8 | MPU_Date[3] );
		ACCEL_Z = (int16_t)(MPU_Date[4]  << 8 | MPU_Date[5] );
		//Temp_Raw= (int16_t)(MPU_Date[6]  << 8 | MPU_Date[7] );
		Gyro_X  = (int16_t)(MPU_Date[8]  << 8 | MPU_Date[9] );
		Gyro_Y  = (int16_t)(MPU_Date[10] << 8 | MPU_Date[11]);
		Gyro_Z  = (int16_t)(MPU_Date[12] << 8 | MPU_Date[12]);
		
		/*Save mpu6050's data into the user's struct*/
		Data->Ax   = ACCEL_X / AccelDivider ;
		Data->Ay   = ACCEL_Y / AccelDivider ;
		Data->Az   = ACCEL_Z / AccelDivider ;
		Data->Gx   = Gyro_X  / GyroDivider  ;
		Data->Gy   = Gyro_Y  / GyroDivider  ;
		Data->Gz   = Gyro_Z  / GyroDivider  ;
		//Data->Temp = (Temp_Raw / 340.0) + 36.53;
	}
	
}

void MPU_ReadRawData(MPU_Data *Data)
{
	if( MpuModuleState == 0 )
	{
		/*Do Nothing if the sensor is not initialzed*/
	}
	else
	{
		/*Local array to save the data read from MPU6050*/
		uint8_t MPU_Date[14] ;
		
		/*variables to save All IMU's data              */
		int16_t ACCEL_X ;
		int16_t ACCEL_Y ;
		int16_t ACCEL_Z ;
		int16_t Temp_Raw;
		int16_t Gyro_X  ;
		int16_t Gyro_Y  ;
		int16_t Gyro_Z  ;
		
		/*Read all MPU6050's Data*/
		HAL_I2C_Mem_Read(GlobalConfig , MPU6050_SLAVE_ADDR , ACCEL_XOUT_H , 1 , MPU_Date , 14 , HAL_MAX_DELAY);
		
		/*Rearrange the mpu6050' data*/
		ACCEL_X = (int16_t)(MPU_Date[0]  << 8 | MPU_Date[1] );
		ACCEL_Y = (int16_t)(MPU_Date[2]  << 8 | MPU_Date[3] );
		ACCEL_Z = (int16_t)(MPU_Date[4]  << 8 | MPU_Date[5] );
		//Temp_Raw= (int16_t)(MPU_Date[6]  << 8 | MPU_Date[7] );
		Gyro_X  = (int16_t)(MPU_Date[8]  << 8 | MPU_Date[9] );
		Gyro_Y  = (int16_t)(MPU_Date[10] << 8 | MPU_Date[11]);
		Gyro_Z  = (int16_t)(MPU_Date[12] << 8 | MPU_Date[12]);
		
		/*Save mpu6050's data into the user's struct*/
		Data->Ax   = ACCEL_X;
		Data->Ay   = ACCEL_Y;
		Data->Az   = ACCEL_Z;
		Data->Gx   = Gyro_X ;
		Data->Gy   = Gyro_Y ;
		Data->Gz   = Gyro_Z ;
		//Data->Temp = (Temp_Raw);
	}
	
}

