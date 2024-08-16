/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#define SMSN_FM_CS_OBC_Pin GPIO_PIN_6
#define SMSN_FM_CS_OBC_GPIO_Port GPIOI
#define MAIN_FM_MISO_Pin GPIO_PIN_4
#define MAIN_FM_MISO_GPIO_Port GPIOB
#define MAIN_FM_CS_Pin GPIO_PIN_3
#define MAIN_FM_CS_GPIO_Port GPIOD
#define MAIN_FM_MOSI_Pin GPIO_PIN_12
#define MAIN_FM_MOSI_GPIO_Port GPIOC
#define MSN_FM_MODE_Pin GPIO_PIN_7
#define MSN_FM_MODE_GPIO_Port GPIOI
#define EN_4V_DCDC_Pin GPIO_PIN_4
#define EN_4V_DCDC_GPIO_Port GPIOI
#define MAIN_FM_SCK_Pin GPIO_PIN_3
#define MAIN_FM_SCK_GPIO_Port GPIOB
#define SMSN_FM_MISO_OBC_Pin GPIO_PIN_5
#define SMSN_FM_MISO_OBC_GPIO_Port GPIOE
#define SMSN_FM_MOSI_OBC_Pin GPIO_PIN_6
#define SMSN_FM_MOSI_OBC_GPIO_Port GPIOE
#define MUX_EN_Pin GPIO_PIN_6
#define MUX_EN_GPIO_Port GPIOG
#define DEBUG_TX_OBC_Pin GPIO_PIN_8
#define DEBUG_TX_OBC_GPIO_Port GPIOE
#define MAG_DRDY_Pin GPIO_PIN_3
#define MAG_DRDY_GPIO_Port GPIOH
#define MAG_INT_Pin GPIO_PIN_15
#define MAG_INT_GPIO_Port GPIOF
#define EN_4V_Pin GPIO_PIN_12
#define EN_4V_GPIO_Port GPIOH
#define EN_3V3_COM_Pin GPIO_PIN_11
#define EN_3V3_COM_GPIO_Port GPIOH
#define CS_MAG_Pin GPIO_PIN_12
#define CS_MAG_GPIO_Port GPIOF
#define MPU_INT_Pin GPIO_PIN_0
#define MPU_INT_GPIO_Port GPIOG
#define DEBUG_RX_OBC_Pin GPIO_PIN_7
#define DEBUG_RX_OBC_GPIO_Port GPIOE
#define CS_MPU_Pin GPIO_PIN_10
#define CS_MPU_GPIO_Port GPIOH
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
