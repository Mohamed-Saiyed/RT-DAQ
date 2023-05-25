/***********************************************************
************************************************************
**														  **		
**	FILENAME	:	DAQ_CAN.c                             **
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
#include "main.h"

#include "IndProximity.h"
#include "SUS_Travel.h"
#include "DAQ_Tasks.h"
#include "Adafruit_BNO055.h"
#include "Encoder.h"

#include "DAQ_CAN.h"


extern CAN_HandleTypeDef hcan1;
extern float VehicleSpeed;
extern double LinAcc[];
extern float ImuAngles[];
extern double GyroBuffer[];
extern uint8_t SystemCalib, GyroCalib , AccelCalib, MgCalib;


static CAN_TxHeaderTypeDef CanMessageWheelsRPM;
static CAN_TxHeaderTypeDef CanMessageImuLinearAccelration;
static CAN_TxHeaderTypeDef CanMessageStrAngle_Speed_TimStamp;
static CAN_TxHeaderTypeDef CanMessageSusTravel;
static CAN_TxHeaderTypeDef CanMessageImuAngles;
static CAN_TxHeaderTypeDef CanMessageImuCalibration;


static uint16_t CanTxTimeStamp = 0 ;
static volatile  uint32_t TimeStamp = 2552357 ;

static HAL_StatusTypeDef HAL_CAN_AddTxMessage_IMUxy(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint32_t *pTxMailbox);
static HAL_StatusTypeDef HAL_CAN_AddTxMessage_IMUz_RollPitch(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint32_t *pTxMailbox);

static int16_t SteeringAngle= -38;

static void CAN_MessageInit(CAN_TxHeaderTypeDef *Message , uint32_t MessageId , uint32_t DataLenght)
{
	Message->StdId = MessageId ;
	Message->ExtId = 0x00;
	Message->IDE   = CAN_ID_STD;
	Message->RTR   = CAN_RTR_DATA;
	Message->DLC   = DataLenght;
	Message->TransmitGlobalTime = DISABLE;
}

void DAQ_CanInitAllMessages(void)
{
	CAN_MessageInit(&CanMessageStrAngle_Speed_TimStamp	,  STR_ANGLE_SPEED_TIMSTAMP_MESSAGE_ID	, STR_ANGLE_SPEED_TIMSTAMP_MESSAGE_DLC);
	CAN_MessageInit(&CanMessageWheelsRPM								,  WHEELS_RPM_MESSAGE_ID		, WHEELS_RPM_MESSAGE_DLC);
	CAN_MessageInit(&CanMessageImuLinearAccelration			,  IMU_IMU_LINEAR_ACCELRATION_MESSAGE_ID			, IMU_IMU_LINEAR_ACCELRATION_MESSAGE_DLC);
	CAN_MessageInit(&CanMessageSusTravel								,  SUS_TRAVEL_MESSAGE_ID		, SUS_TRAVEL_MESSAGE_DLC);
	CAN_MessageInit(&CanMessageImuAngles								,  IMU_ANGLES_Gz_MESSAGE_ID		, IMU_ANGLES_Gz_MESSAGE_DLC);
	CAN_MessageInit(&CanMessageImuCalibration						,  IMU_CALIBRATION_MESSAGE_ID		, IMU_CALIBRATION_MESSAGE_DLC);
}

void DAQ_CanSend_RPMs(void)
{
	uint16_t RPMs[4] = {5367 ,5436,4255,2467};
	uint8_t RPMsToSend[8];
	uint32_t RpmMailBox;
	
	uint32_t RpmMessageTimeout = 50;
	uint32_t TickStart		   = 0;
	
	//IndProximity_GetWheelsRPM(RPMs);
	
	for(uint8_t LocalIndex = 0 ; LocalIndex < 4 ; LocalIndex++)
	{
		RPMsToSend[2 * (LocalIndex)]     = (uint8_t)(RPMs[LocalIndex]  & 0x00FF);
		RPMsToSend[(2 * LocalIndex) + 1] = (uint8_t)((RPMs[LocalIndex] & 0xFF00) >> 8);
	}
	
	HAL_CAN_AddTxMessage(&hcan1, &CanMessageWheelsRPM, RPMsToSend, &RpmMailBox);
	
	TickStart = HAL_GetTick();
	
	while(HAL_CAN_IsTxMessagePending(&hcan1, RpmMailBox) == 1)
	{
		if((HAL_GetTick() - TickStart) >= RpmMessageTimeout)
		{
			break;
		}
	}
	
}

void DAQ_CanSend_SUSTravel(void)
{
	uint8_t SUSTravel[4];
	uint32_t SUSTravelMailBox;
	
	uint32_t SUSTravelMessageTimeout = 50;
	uint32_t TickStart		   = 0;
	
	SUSTravel_GetData(SUSTravel , ALL_TRAVEL_SESNORS);
	
	HAL_CAN_AddTxMessage(&hcan1, &CanMessageSusTravel, SUSTravel, &SUSTravelMailBox);
	
	TickStart = HAL_GetTick();
	
	while(HAL_CAN_IsTxMessagePending(&hcan1, SUSTravelMailBox) == 1)
	{
		if((HAL_GetTick() - TickStart) >= SUSTravelMessageTimeout)
		{
			break;
		}
	}
	
}


void DAQ_CanSendSpeed_StrAngle_TimStamp(void)
{
	uint32_t Speed_StrAngle_TimStampMailBox;
	
	uint32_t Speed_StrAngleTimSTampTimeout = 50;
	uint32_t TickStart    = 0;
	uint16_t SteeringAngleToSend;
	float VehicleSpeedToSend;
	
	//Encoder_GetSteeringAngle(&SteeringAngle);
	SteeringAngleToSend = SteeringAngle + 180;
	VehicleSpeedToSend = VehicleSpeed * 100;
	
	//CanTxTimeStamp = HAL_CAN_GetTxTimestamp(&hcan1 , Speed_StrAngle_TimStampMailBox);
	
	TimeStamp++;
	
	uint8_t Speed_StrAngle_TimStamp[8] = {((uint16_t)VehicleSpeedToSend & 0x00FF), (((uint16_t)VehicleSpeedToSend & 0xFF00) >> 8) ,
											(SteeringAngleToSend & 0x00FF) , ((SteeringAngleToSend & 0xFF00) >> 8),
										  (TimeStamp & 0x000000FF) , ((TimeStamp & 0x0000FF00) >> 8),
											((TimeStamp & 0x00FF0000) >> 16) , ((TimeStamp & 0xFF000000) >> 24)};
	
	
	HAL_CAN_AddTxMessage(&hcan1, &CanMessageStrAngle_Speed_TimStamp, Speed_StrAngle_TimStamp, &Speed_StrAngle_TimStampMailBox);
	
	
	TickStart = HAL_GetTick();
	
	while(HAL_CAN_IsTxMessagePending(&hcan1, Speed_StrAngle_TimStampMailBox) == 1)
	{
		if((HAL_GetTick() - TickStart) >= Speed_StrAngleTimSTampTimeout)
		{
			break;
		}
	}
}


void DAQ_CanSendImuAngles_Gz(void)
{
	uint32_t AnglesMailbox;
  float AnglesBuffer[3];
	uint8_t AnglesCanBuffer[8];
	uint32_t IMUAnlgesMessageTimeout = 50;
	float GyroZ = 0;
	uint32_t TickStart		   = 0;
	
	for(uint8_t Local_Index = 0 ; Local_Index < 3 ; Local_Index++)
	{
		AnglesBuffer[Local_Index] = ((ImuAngles[Local_Index] + 360) * 100);
	}
	
	for(uint8_t Local_Index = 0 ; Local_Index < 3 ; Local_Index++)
	{
		AnglesCanBuffer[2 * Local_Index] = (uint8_t)((uint16_t)AnglesBuffer[Local_Index] & 0x00FF);
		
		AnglesCanBuffer[(2 * Local_Index) + 1] = (uint8_t)(((uint16_t)AnglesBuffer[Local_Index] & 0xFF00) >> 8);
	}
	
	GyroZ = ((GyroBuffer[2] + 3600) * 10); 
	
	AnglesCanBuffer[6] = (uint8_t)((uint16_t) GyroZ & 0x00FF);
	AnglesCanBuffer[7] = (uint8_t)(((uint16_t)GyroZ & 0xFF00) >> 8);
	
	
	HAL_CAN_AddTxMessage(&hcan1, &CanMessageImuAngles, AnglesCanBuffer, &AnglesMailbox);
		
	TickStart = HAL_GetTick();
	
		while(HAL_CAN_IsTxMessagePending(&hcan1, AnglesMailbox) == 1)
		{
			if((HAL_GetTick() - TickStart) >= IMUAnlgesMessageTimeout)
			{
				break;
			}
		}
	
		
	
}

void DAQ_CanSendIMUCalibration(void)
{
	uint32_t IMUClibrationMailBox;
	uint32_t IMUClibrationMessageTimeout = 50;
	uint32_t TickStart    = 0;
	uint8_t IMUClibrationValues[4]		= {SystemCalib,AccelCalib,GyroCalib,MgCalib};  
	
	
	HAL_CAN_AddTxMessage(&hcan1, &CanMessageImuCalibration, IMUClibrationValues, &IMUClibrationMailBox);
	
	TickStart = HAL_GetTick();
	
	while(HAL_CAN_IsTxMessagePending(&hcan1, IMUClibrationMailBox) == 1)
	{
		if((HAL_GetTick() - TickStart) >= IMUClibrationMessageTimeout)
		{
			break;
		}
	}
	
}


void DAQ_CANSendIMULinearAccelration(void)
{
	float 	 LinearAccBuffer[3];
	uint8_t  LinearAccCanBuffer[6];
	uint32_t IMULinearAccelrationMailBox;
	uint32_t IMULinearAccelrationMailBoxTimeout = 50;
	uint32_t TickStart    = 0;
	
	for(uint8_t Local_Index = 0 ; Local_Index < 3 ; Local_Index++)
	{
		LinearAccBuffer[Local_Index] = ((LinAcc[Local_Index] + 360) * 100);
	}
	
	for(uint8_t Local_Index = 0 ; Local_Index < 3 ; Local_Index++)
	{
		LinearAccCanBuffer[2 * Local_Index] = (uint8_t)((uint16_t)LinearAccBuffer[Local_Index] & 0x00FF);
		
		LinearAccCanBuffer[(2 * Local_Index) + 1] = (uint8_t)(((uint16_t)LinearAccBuffer[Local_Index] & 0xFF00) >> 8);
	}
	
	HAL_CAN_AddTxMessage(&hcan1, &CanMessageImuLinearAccelration, LinearAccCanBuffer , &IMULinearAccelrationMailBox);
	
	TickStart = HAL_GetTick();
	
	while(HAL_CAN_IsTxMessagePending(&hcan1, IMULinearAccelrationMailBox) == 1)
	{
		if((HAL_GetTick() - TickStart) >= IMULinearAccelrationMailBoxTimeout)
		{
			break;
		}
	}
}
