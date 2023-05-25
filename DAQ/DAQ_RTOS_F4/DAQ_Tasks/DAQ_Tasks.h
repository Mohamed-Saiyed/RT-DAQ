#ifndef DAQ_TASKS_H
#define DAQ_TASKS_H


extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern CAN_HandleTypeDef hcan1;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim1;

void Vehicle_CreateAllTasks(void);
void VehicleGetSteeringAngle(void * pvParameters);
void VehicleGetSpeed(void * pvParameters);
void VehicleGetTravelReading(void * pvParameters);
void VehicleGetImuData(void * pvParameters);
void VehicleCanTransmitAllData(void * pvParameters);
void VehicleLogAllData(void * pvParameters);
#endif /*DAQ_TASKS_H*/