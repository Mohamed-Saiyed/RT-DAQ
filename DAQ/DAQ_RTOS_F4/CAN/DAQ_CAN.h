/***********************************************************
************************************************************
**														  **		
**	FILENAME	:	DAQ_CAN.h                             **
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
**	DESCRIPTION : Data Acquisition's CAN Source File      **
**                                                        **
**	MAY BE CHANGED BY USER : NO                           **
**                                                        **
***********************************************************/	
#ifndef	DAQ_CAN_H
#define DAQ_CAN_H
	
#define WHEELS_RPM_MESSAGE_ID										0x20
#define IMU_IMU_LINEAR_ACCELRATION_MESSAGE_ID		0x52
#define STR_ANGLE_SPEED_TIMSTAMP_MESSAGE_ID			0x01
#define SUS_TRAVEL_MESSAGE_ID										0x30
#define IMU_ANGLES_Gz_MESSAGE_ID								0x25
#define IMU_CALIBRATION_MESSAGE_ID							0x35

#define WHEELS_RPM_MESSAGE_DLC									0x08
#define IMU_IMU_LINEAR_ACCELRATION_MESSAGE_DLC	0x06
#define STR_ANGLE_SPEED_TIMSTAMP_MESSAGE_DLC		0x08
#define SUS_TRAVEL_MESSAGE_DLC									0x04
#define IMU_ANGLES_Gz_MESSAGE_DLC								0x08
#define IMU_CALIBRATION_MESSAGE_DLC							0x04

void DAQ_CanInitAllMessages(void);
void DAQ_CanSend_RPMs(void);
void DAQ_CanSend_SUSTravel(void);
void DAQ_CanSendIMUCalibration(void);
void DAQ_CANSendIMULinearAccelration(void);
void DAQ_CanSendImuAngles_Gz(void);
void DAQ_CanSendSpeed_StrAngle_TimStamp(void);

#endif /*DAQ_CAN_H*/


