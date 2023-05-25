/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"
#include "MPU6050.h"
#include "Kalman.h"
//#include "FIR.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
void Kalman_Setup(void);
void Kalman_MainFunction(void);
void IMU_GetError(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 
float Acceleration_angle[3];
float Gyro_angle[3];
float Total_angle[3];

float Acceleration_angle_nofilter[2];
float Gyro_angle_nofilter[2];
float Total_angle_nofilter[2];

float speed;
int16_t anglespitch;
int16_t anglesroll;
int16_t anglespitch_nofilter;
double accX, accY, accZ;
double gyroX , gyroY, gyroZ;
int16_t tempRaw;

float ErrorAX = 0 ;
float ErrorAY = 0 ;


volatile double gyroXangle, gyroYangle , gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
float kalAngleX, kalAngleY ,kalAngleZ ; // Calculated angle using a Kalman filter
float gyro_x_cal ;
float gyro_y_cal ;
float gyro_z_cal ;
float elapsedTime, time, timePrev;

float RAD_TO_DEG = 180/3.141592654;

int16_t KalmanAngle = 0 ;
int16_t CompAngle   = 0 ;

MPU_Data Data;
MPU_Data DataOut;
MPU_Data RawData;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */



  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	MPU_Init(&hi2c1 , IMU_ACCEL_SCALE_2g , IMU_GYRO_SCALE_250);
	IMU_GetError();
	Kalman_Setup();
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
			Kalman_MainFunction();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

Kalman_Angle Angle_Pitch;
Kalman_Angle Angle_Roll;
void Kalman_Setup(void)
{
	Kalamn_InitAngle(&Angle_Pitch);
	Kalamn_InitAngle(&Angle_Roll);
	uint8_t MPU_Date[14] ;
		
		/*Read all MPU6050's Data*/
		HAL_I2C_Mem_Read(&hi2c1 , MPU6050_SLAVE_ADDR , ACCEL_XOUT_H , 1 , MPU_Date , 14 , HAL_MAX_DELAY);
		
		/*Rearrange the mpu6050' data*/
		accX = (int16_t)(MPU_Date[0]  << 8 | MPU_Date[1] );
		accY = (int16_t)(MPU_Date[2]  << 8 | MPU_Date[3] );
		accZ = (int16_t)(MPU_Date[4]  << 8 | MPU_Date[5] );
		//Temp_Raw= (int16_t)(MPU_Date[6]  << 8 | MPU_Date[7] );
		gyroX  = (int16_t)(MPU_Date[8]  << 8 | MPU_Date[9] );
		gyroY  = (int16_t)(MPU_Date[10] << 8 | MPU_Date[11]);
		gyroZ  = (int16_t)(MPU_Date[12] << 8 | MPU_Date[12]);
	
	  gyroX	-= gyro_x_cal ;		
	  gyroY -= gyro_y_cal ;
	  gyroZ	-= gyro_z_cal ;
	
		double pitch   = atan(-accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;//+ ErrorAY;
    double roll = (atan2(-accX, accZ))* RAD_TO_DEG ;//-  ErrorAX ;
		roll    -= ErrorAY ;
		pitch   -= ErrorAX ;
		
		Kalman_setAngle(&Angle_Pitch,pitch);
		Kalman_setAngle(&Angle_Roll,roll);
	  gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;
	  
	  time = HAL_GetTick();
}


void Kalman_MainFunction(void)
{
		uint8_t MPU_Date[14] ;
		
		/*Read all MPU6050's Data*/
		HAL_I2C_Mem_Read(&hi2c1 , MPU6050_SLAVE_ADDR , ACCEL_XOUT_H , 1 , MPU_Date , 14 , HAL_MAX_DELAY);
		
		/*Rearrange the mpu6050' data*/
		accX = (int16_t)(MPU_Date[0]  << 8 | MPU_Date[1] );
		accY = (int16_t)(MPU_Date[2]  << 8 | MPU_Date[3] );
		accZ = (int16_t)(MPU_Date[4]  << 8 | MPU_Date[5] );
		//Temp_Raw= (int16_t)(MPU_Date[6]  << 8 | MPU_Date[7] );
		gyroX  = (int16_t)(MPU_Date[8]  << 8 | MPU_Date[9] );
		gyroY  = (int16_t)(MPU_Date[10] << 8 | MPU_Date[11]);
		gyroZ  = (int16_t)(MPU_Date[12] << 8 | MPU_Date[12]);
	
		gyroX	-= gyro_x_cal ;
		gyroY -= gyro_y_cal ;
	   gyroZ	-= gyro_z_cal ;
	
       timePrev = time;
	   time = HAL_GetTick();
		 elapsedTime = (time - timePrev) / 1000;
		 
		
		double pitch   = atan(-accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;//+ ErrorAY;
    double roll = (atan2(-accX, accZ))* RAD_TO_DEG ;//-  ErrorAX ;
		roll    -= ErrorAY;
	  pitch   -= ErrorAX ;
		
	double gyroXrate = gyroX / 131.0; // Convert to deg/s
	double gyroYrate = gyroY / 131.0; // Convert to deg/s
	double gyroZrate = gyroZ / 131.0; // Convert to deg/s	
	
	gyroXangle += gyroXrate * elapsedTime; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * elapsedTime;
	gyroZangle += gyroZrate * elapsedTime;
	
	
	 if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) 
	{
    Kalman_setAngle(&Angle_Pitch,pitch);
    kalAngleY = pitch;
    gyroYangle = pitch;
  }
	else
	{
    kalAngleX = Kalman_getAngle(&Angle_Pitch, pitch, gyroYrate, elapsedTime); // Calculate the angle using a Kalman filter
	}

  if (fabs(kalAngleY) > 90)
	{
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
	}
  kalAngleY = Kalman_getAngle(&Angle_Roll,roll , gyroXrate, elapsedTime); // Calculate the angle using a Kalman filter
  
	
  compAngleX = 0.95 * (compAngleX + gyroXrate * elapsedTime) + 0.05 * (roll);
  
  compAngleY = 0.95 * (compAngleY + gyroYrate * elapsedTime) + 0.05 * (pitch);

  if (gyroXangle < -180 || gyroXangle > 180)
	{
    gyroXangle = kalAngleX;
	}
  if (gyroYangle < -180 || gyroYangle > 180)
	{
    gyroYangle = kalAngleY;
	}

}



void IMU_GetError(void)
{
		uint8_t MPU_Date[6] ;
		
		/*Read all MPU6050's Data*/
		for(uint16_t Local = 0 ; Local < 2000 ; Local++)
		{
			 uint8_t MPU_Date[6] ;
		
		 /*Read all MPU6050's Data*/
			HAL_I2C_Mem_Read(&hi2c1 , MPU6050_SLAVE_ADDR , GYRO_XOUT_H , 1 , MPU_Date , 6 , HAL_MAX_DELAY);
			
			/*Rearrange the mpu6050' data*/
			float gyrox = (int16_t)(MPU_Date[0]  << 8 | MPU_Date[1] );
			float gyroy = (int16_t)(MPU_Date[2]  << 8 | MPU_Date[3] );
			float gyroz = (int16_t)(MPU_Date[4]  << 8 | MPU_Date[5] );
		
			gyro_x_cal	 += gyrox ;
			gyro_y_cal   += gyroy ;
			gyro_z_cal   += gyroz ;

		}
		
		gyro_x_cal /= 2000;
		gyro_y_cal /= 2000;
		gyro_z_cal /= 2000;
		
		for(uint16_t Local = 0 ; Local < 200 ; Local++)
		{
			 uint8_t MPU_Date[6] ;
		
		 /*Read all MPU6050's Data*/
			HAL_I2C_Mem_Read(&hi2c1 , MPU6050_SLAVE_ADDR , ACCEL_XOUT_H , 1 , MPU_Date , 6 , HAL_MAX_DELAY);
			
			/*Rearrange the mpu6050' data*/
			float accelx = (int16_t)(MPU_Date[0]  << 8 | MPU_Date[1] );
			float accely = (int16_t)(MPU_Date[2]  << 8 | MPU_Date[3] );
			float accelz = (int16_t)(MPU_Date[4]  << 8 | MPU_Date[5] );
			
			double pitch   = atan(-accely / sqrt(accelx * accelx + accelz * accelz)) * RAD_TO_DEG;//+ ErrorAY;
      double roll = (atan2(-accelx, accelz))* RAD_TO_DEG ;//-  ErrorAX ;
			
			ErrorAX += pitch;
			ErrorAY += roll;

		}
		
		ErrorAX /= 200; 
		ErrorAY /= 200;
}


/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
