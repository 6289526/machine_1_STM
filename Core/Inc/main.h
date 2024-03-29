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
#include "stm32f2xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Servo1_Pin GPIO_PIN_15
#define Servo1_GPIO_Port GPIOB
#define PI_TX_Pin GPIO_PIN_8
#define PI_TX_GPIO_Port GPIOD
#define PI_RX_Pin GPIO_PIN_9
#define PI_RX_GPIO_Port GPIOD
#define DC_Front_Left_Pin GPIO_PIN_6
#define DC_Front_Left_GPIO_Port GPIOC
#define DC_Front_Right_Pin GPIO_PIN_7
#define DC_Front_Right_GPIO_Port GPIOC
#define DC_Back_Left_Pin GPIO_PIN_8
#define DC_Back_Left_GPIO_Port GPIOC
#define DC_Back_Right_Pin GPIO_PIN_9
#define DC_Back_Right_GPIO_Port GPIOC
#define DC_Pantagraph_Pin GPIO_PIN_8
#define DC_Pantagraph_GPIO_Port GPIOA
#define Servo_Right_Pin GPIO_PIN_9
#define Servo_Right_GPIO_Port GPIOA
#define Servo_Left_Pin GPIO_PIN_10
#define Servo_Left_GPIO_Port GPIOA
#define TPIP_TX_Pin GPIO_PIN_5
#define TPIP_TX_GPIO_Port GPIOD
#define TPIP_RX_Pin GPIO_PIN_6
#define TPIP_RX_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
