/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define ICM42688_SCL_Pin GPIO_PIN_13
#define ICM42688_SCL_GPIO_Port GPIOC
#define ICM42688_SDA_Pin GPIO_PIN_14
#define ICM42688_SDA_GPIO_Port GPIOC
#define U_ADC_Pin GPIO_PIN_2
#define U_ADC_GPIO_Port GPIOA
#define W_ADC_Pin GPIO_PIN_3
#define W_ADC_GPIO_Port GPIOA
#define TEMP_ADC_Pin GPIO_PIN_6
#define TEMP_ADC_GPIO_Port GPIOA
#define VBAT_ADC_Pin GPIO_PIN_7
#define VBAT_ADC_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOB
#define RS485_EN_Pin GPIO_PIN_12
#define RS485_EN_GPIO_Port GPIOB
#define FD6288Q_LIN3_Pin GPIO_PIN_13
#define FD6288Q_LIN3_GPIO_Port GPIOB
#define FD6288Q_LIN2_Pin GPIO_PIN_14
#define FD6288Q_LIN2_GPIO_Port GPIOB
#define FD6288Q_LIN1_Pin GPIO_PIN_15
#define FD6288Q_LIN1_GPIO_Port GPIOB
#define FD6288Q_HIN3_Pin GPIO_PIN_8
#define FD6288Q_HIN3_GPIO_Port GPIOA
#define FD6288Q_HIN2_Pin GPIO_PIN_9
#define FD6288Q_HIN2_GPIO_Port GPIOA
#define FD6288Q_HIN1_Pin GPIO_PIN_10
#define FD6288Q_HIN1_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SPI_CSN_Pin GPIO_PIN_15
#define SPI_CSN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
