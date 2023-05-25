/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SUS_1_Pin GPIO_PIN_0
#define SUS_1_GPIO_Port GPIOC
#define SUS_2_Pin GPIO_PIN_1
#define SUS_2_GPIO_Port GPIOC
#define AN_1_Pin GPIO_PIN_2
#define AN_1_GPIO_Port GPIOC
#define AN_2_Pin GPIO_PIN_3
#define AN_2_GPIO_Port GPIOC
#define Enc_Channel_A_Pin GPIO_PIN_0
#define Enc_Channel_A_GPIO_Port GPIOA
#define Enc_Channel_B_Pin GPIO_PIN_1
#define Enc_Channel_B_GPIO_Port GPIOA
#define AN_3_DI_4_Pin GPIO_PIN_2
#define AN_3_DI_4_GPIO_Port GPIOA
#define BNO055_RST_Pin GPIO_PIN_3
#define BNO055_RST_GPIO_Port GPIOA
#define SUS_3_Pin GPIO_PIN_6
#define SUS_3_GPIO_Port GPIOA
#define SUS_4_Pin GPIO_PIN_7
#define SUS_4_GPIO_Port GPIOA
#define Pressure_Sensor1_Pin GPIO_PIN_4
#define Pressure_Sensor1_GPIO_Port GPIOC
#define Pressure_Sensor2_Pin GPIO_PIN_5
#define Pressure_Sensor2_GPIO_Port GPIOC
#define T_est_LED_F4_Pin GPIO_PIN_9
#define T_est_LED_F4_GPIO_Port GPIOC
#define RR_RPM_Pin GPIO_PIN_8
#define RR_RPM_GPIO_Port GPIOA
#define RL_RPM_Pin GPIO_PIN_9
#define RL_RPM_GPIO_Port GPIOA
#define FL_RPM_Pin GPIO_PIN_10
#define FL_RPM_GPIO_Port GPIOA
#define FR_RPM_Pin GPIO_PIN_11
#define FR_RPM_GPIO_Port GPIOA
#define SD_CardDetect_Pin GPIO_PIN_12
#define SD_CardDetect_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DI_2_Pin GPIO_PIN_15
#define DI_2_GPIO_Port GPIOA
#define DI_3_Pin GPIO_PIN_4
#define DI_3_GPIO_Port GPIOB
#define DI_1_Pin GPIO_PIN_5
#define DI_1_GPIO_Port GPIOB
#define BNO055_SCL_Pin GPIO_PIN_6
#define BNO055_SCL_GPIO_Port GPIOB
#define BNO055_SDA_Pin GPIO_PIN_7
#define BNO055_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
