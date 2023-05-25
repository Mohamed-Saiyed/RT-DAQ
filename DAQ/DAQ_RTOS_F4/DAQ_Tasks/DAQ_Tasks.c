#include "main.h"

#include "FreeRTOS.h"
#include "task.h"


#include "IndProximity.h"
#include "SUS_Travel.h"
#include "DAQ_CAN.h"
#include "Adafruit_BNO055.h"
#include "Encoder.h"
#include "DataLogging.h"
#include "DAQ_Tasks.h"




TaskHandle_t TASK1_Handler = NULL;
TaskHandle_t TASK2_Handler = NULL;
TaskHandle_t TASK3_Handler = NULL;
TaskHandle_t TASK4_Handler = NULL;
TaskHandle_t TASK5_Handler = NULL;
TaskHandle_t TASK6_Handler = NULL;
TaskHandle_t TASK7_Handler = NULL;
float VehicleSpeed = 114.6 ;
Quaternion q;
DAQStrcut DataAcquisition;
uint8_t SystemCalib, GyroCalib , AccelCalib, MgCalib = 0;
float ImuAngles[3] = {0.f};
double LinAcc[3]  = {-0.578654 , -75.35584 , .968557};
double GyroBuffer[3] = {0.f};

void Vehicle_CreateAllTasks(void)
{
	xTaskCreate(VehicleGetSteeringAngle , "TASK3" , configMINIMAL_STACK_SIZE , NULL , 6 , &TASK1_Handler);
	xTaskCreate(IndProximity_MainFunction , "TASK1" , configMINIMAL_STACK_SIZE , NULL , 5 , &TASK2_Handler);
	xTaskCreate(VehicleGetTravelReading , "TASK2" , configMINIMAL_STACK_SIZE , NULL , 4 , &TASK3_Handler);
	xTaskCreate(VehicleGetSpeed , "TASK4" , configMINIMAL_STACK_SIZE , NULL , 2 , &TASK4_Handler);
	xTaskCreate(VehicleGetImuData , "TASK5" , configMINIMAL_STACK_SIZE , NULL , 3 , &TASK5_Handler);
//	xTaskCreate(VehicleLogAllData,"TASK7",1024*2 , NULL , 2 , &TASK7_Handler);
	xTaskCreate(VehicleCanTransmitAllData , "TASK6" , configMINIMAL_STACK_SIZE , NULL , 1 , &TASK6_Handler);	
	
}




void VehicleGetSteeringAngle(void * pvParameters)
{
	TickType_t xLastWakeTime;
	vTaskSetApplicationTaskTag(NULL , (void* )5);
	Encoder_Start();
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//HAL_GPIO_WritePin(GPIOB , GPIO_PIN_0 , GPIO_PIN_SET);
		Encoder_MainFuction();
		//HAL_GPIO_WritePin(GPIOB , GPIO_PIN_0 , GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime,1);
	}
}

void VehicleGetSpeed(void * pvParameters)
{
	TickType_t xLastWakeTime;
	vTaskSetApplicationTaskTag(NULL , (void* )4);
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//HAL_GPIO_WritePin(GPIOB , GPIO_PIN_0 , GPIO_PIN_SET);
		IndProximity_GetVehicleSpeed(&VehicleSpeed);
	  //HAL_GPIO_WritePin(GPIOB , GPIO_PIN_0 , GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime , 4);
	}

}

void VehicleGetTravelReading(void * pvParameters)
{
	TickType_t xLastWakeTime;
	vTaskSetApplicationTaskTag(NULL , (void* )3);
	SUSTravel_StartDMA(&hadc1);
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//HAL_GPIO_WritePin(GPIOB , GPIO_PIN_0 , GPIO_PIN_SET);
		SUSTravel_MainFunction();
		//HAL_GPIO_WritePin(GPIOB , GPIO_PIN_0 , GPIO_PIN_RESET);
		
		vTaskDelayUntil(&xLastWakeTime , 2);
		
	}
}

void VehicleGetImuData(void * pvParameters)
{
	
	TickType_t xLastWakeTime;
	vTaskSetApplicationTaskTag(NULL , (void* )2);
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		
		Adafruit_BNO055_GetCalibration(&SystemCalib, &GyroCalib, &AccelCalib, &MgCalib);
		Adafruit_BNO055_GetQuat(&q);
		Adafruit_BNO055_UpdateEuler(q.w , q.x , q.y ,q.z , ImuAngles);
	  Adafruit_BNO055_GetVector(VECTOR_LINEARACCEL , LinAcc);
		
		for(uint8_t Local_Index = 0 ; Local_Index < 3 ; Local_Index++)
		{
			DataAcquisition.LinAcc[Local_Index] = LinAcc[Local_Index];
		}
		for(uint8_t Local_Index = 0 ; Local_Index < 3 ; Local_Index++)
		{
			DataAcquisition.ImuAngles[Local_Index] = ImuAngles[Local_Index];
		}
		
		
		vTaskDelayUntil(&xLastWakeTime , 8);
	}
}


void VehicleCanTransmitAllData(void * pvParameters)
{
	TickType_t xLastWakeTime;
	vTaskSetApplicationTaskTag(NULL , (void* )1);
	HAL_CAN_Start(&hcan1);
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{

		DAQ_CANSendIMULinearAccelration();
		DAQ_CanSend_RPMs();	
	  DAQ_CanSendSpeed_StrAngle_TimStamp();
		DAQ_CanSendImuAngles_Gz();
   	DAQ_CanSend_SUSTravel();
	  DAQ_CanSendIMUCalibration();
		
		vTaskDelayUntil(&xLastWakeTime,100);
	}
	
}

void VehicleLogAllData(void * pvParameters)
{
	TickType_t xLastWakeTime;
	vTaskSetApplicationTaskTag(NULL , (void* )1);
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		taskENTER_CRITICAL();
		DataAcquisition.AvrageSpeed = 40.65;
		DataAcquisition.ChasisSpeed = 41.47;
		DataAcquisition.WheelsRPM[0] = 1548;
		DataAcquisition.WheelsRPM[1] = 1547;
		DataAcquisition.WheelsRPM[2] = 1524;
		DataAcquisition.WheelsRPM[3] = 1534;
		DataAcquisition.WheelsTravel[0] = 20.9;
		DataAcquisition.WheelsTravel[1] = 20.9;
		DataAcquisition.WheelsTravel[2] = 20.9;
		DataAcquisition.WheelsTravel[3] = 20.9;
		DataAcquisition.BrakeSwitch = 1;
		DataAcquisition.EngineRPM = 8574;
		DataAcquisition.EngineECT = 78;
		DataAcquisition.latitude = 30.06656583;
		DataAcquisition.Longitude = 31.02895648;
		DataAcquisition.SteeringAngle = -127;
		DataAcquisition.ThrottlePosition = 37.8;
		
		DataLogging_LogDAQ(&DataAcquisition);
		taskEXIT_CRITICAL();
		vTaskDelayUntil(&xLastWakeTime,20);
	}
	
}




