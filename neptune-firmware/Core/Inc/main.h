/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define ARM_MATH_CM4
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RF_10O_Pin GPIO_PIN_0
#define RF_10O_GPIO_Port GPIOC
#define RF_10O_EXTI_IRQn EXTI0_IRQn
#define RF_RESET_Pin GPIO_PIN_1
#define RF_RESET_GPIO_Port GPIOC
#define RF_CE_Pin GPIO_PIN_2
#define RF_CE_GPIO_Port GPIOC
#define SD_CE_Pin GPIO_PIN_3
#define SD_CE_GPIO_Port GPIOC
#define S1_FBK_Pin GPIO_PIN_2
#define S1_FBK_GPIO_Port GPIOA
#define S2_FBK_Pin GPIO_PIN_3
#define S2_FBK_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define IMU_MISO_Pin GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define IMU_MOSI_Pin GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOA
#define ACC_CE_Pin GPIO_PIN_0
#define ACC_CE_GPIO_Port GPIOB
#define GYR_CE_Pin GPIO_PIN_1
#define GYR_CE_GPIO_Port GPIOB
#define MAG_CE_Pin GPIO_PIN_2
#define MAG_CE_GPIO_Port GPIOB
#define RF_SCK_Pin GPIO_PIN_13
#define RF_SCK_GPIO_Port GPIOB
#define RF_MISO_Pin GPIO_PIN_14
#define RF_MISO_GPIO_Port GPIOB
#define RF_MOSI_Pin GPIO_PIN_15
#define RF_MOSI_GPIO_Port GPIOB
#define SD_SCK_Pin GPIO_PIN_10
#define SD_SCK_GPIO_Port GPIOC
#define SD_MISO_Pin GPIO_PIN_11
#define SD_MISO_GPIO_Port GPIOC
#define SD_MOSI_Pin GPIO_PIN_12
#define SD_MOSI_GPIO_Port GPIOC
#define S1_CTRL_Pin GPIO_PIN_4
#define S1_CTRL_GPIO_Port GPIOB
#define S2_CTRL_Pin GPIO_PIN_5
#define S2_CTRL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define Accel_Sensor 1
#define Gyro_Sensor 2
#define Mag_Sensor 4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
