/***********************************************************
************************************************************
**														  **		
**	FILENAME	:	MPU6050.h                             **
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

#ifndef _MPU6050_H
#define _MPU6050_H

/*****************************************************************/
/*				    	Include Headers					         */
/*****************************************************************/

#include "main.h"

/*****************************************************************/
/*				        Macros Definition       		         */
/*****************************************************************/

/* MPU6050 I2C Slave Address */ 
#define MPU6050_SLAVE_ADDR	0xD0

/* MPU6050 Private registers defentions */
#define WHO_AM_I			0x75
#define ACCEL_XOUT_H	 	0x3B
#define GYRO_XOUT_H 	 	0x43
#define PWR_MGMT_1  	 	0x6B
#define SMPLRT_DIV  	 	0x19
#define ACCEL_CONFIG	 	0x1C
#define GYRO_CONFIG 	 	0x1B
#define CONFIG 			 	0x1A

/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_DIVIDER_250		((float) 131.0)
#define MPU6050_GYRO_DIVIDER_500		((float) 65.5)
#define MPU6050_GYRO_DIVIDER_1000		((float) 32.8)
#define MPU6050_GYRO_DIVIDER_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_DIVIDER_2			((float) 16384.0)
#define MPU6050_ACCE_DIVIDER_4			((float) 8192.0)
#define MPU6050_ACCE_DIVIDER_8			((float) 4096.0)
#define MPU6050_ACCE_DIVIDER_16		    ((float) 2048.0)

/*****************************************************************/
/*				        Types Definition        		         */
/*****************************************************************/

/* MPU6050's Gyroscope Scale range Enum */
typedef uint8_t IMU_GyroScaleRange;
#define IMU_GYRO_SCALE_250				((IMU_GyroScaleRange) 0x00U)
#define IMU_GYRO_SCALE_500				((IMU_GyroScaleRange) 0x01U)
#define IMU_GYRO_SCALE_1000             ((IMU_GyroScaleRange) 0x02U)
#define IMU_GYRO_SCALE_2000             ((IMU_GyroScaleRange) 0x03U)

/* MPU6050's Acclerometer Scale range Enum */
typedef uint8_t IMU_AccelScaleRange;
#define IMU_ACCEL_SCALE_2g				((IMU_AccelScaleRange) 0x00U)
#define IMU_ACCEL_SCALE_4g				((IMU_AccelScaleRange) 0x01U)
#define IMU_ACCEL_SCALE_8g              ((IMU_AccelScaleRange) 0x02U)
#define IMU_ACCEL_SCALE_16g             ((IMU_AccelScaleRange) 0x03U)


/*
** This strcut contains the the acclerometer, Gyroscope data and Temperature data
** Read from the IMU
*/
typedef struct
{
	/* Accelerometer X-axis data */
	float Ax ;
	
	/* Accelerometer Y-axis data */
	float Ay ;
	 
	/* Accelerometer Z-axis data */
	float Az ;
	
	/* Gyroscope X-axis data */
	float Gx ;
	
	/* Gyroscope Y-axis data */
	float Gy ;
	
	/* Gyroscope Z-axis data */
	float Gz ;
	
	/* Temperature data */
	float Temp ;

}MPU_Data;

/*****************************************************************/
/*				        Functions Prototype        		         */
/*****************************************************************/

void MPU_Init(I2C_HandleTypeDef *hi2c ,IMU_AccelScaleRange AccelScale ,IMU_GyroScaleRange GyroScale);
void MPU_AccelRead(MPU_Data *AccelData);
void MPU_GyroRead(MPU_Data *GyroData);
void MPU_ReadAll(MPU_Data *Data);
void MPU_ReadRawData(MPU_Data *Data);


#endif /*_MPU6050_H*/

