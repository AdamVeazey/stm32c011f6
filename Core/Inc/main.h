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
#include "stm32c0xx_hal.h"

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
#define CN5_23_ADC_IN0_Pin GPIO_PIN_0
#define CN5_23_ADC_IN0_GPIO_Port GPIOA
#define CN1_2_USART2_TX_Pin GPIO_PIN_2
#define CN1_2_USART2_TX_GPIO_Port GPIOA
#define CN5_18_USART2_RX_Pin GPIO_PIN_3
#define CN5_18_USART2_RX_GPIO_Port GPIOA
#define CN5_26_SPI1_SCK_Pin GPIO_PIN_5
#define CN5_26_SPI1_SCK_GPIO_Port GPIOA
#define CN5_16_SPI_MISO_Pin GPIO_PIN_6
#define CN5_16_SPI_MISO_GPIO_Port GPIOA
#define CN5_11_SPI1_MOSI_Pin GPIO_PIN_7
#define CN5_11_SPI1_MOSI_GPIO_Port GPIOA
#define CN5_15_Joystick_Pin GPIO_PIN_8
#define CN5_15_Joystick_GPIO_Port GPIOA
#define CN5_6_USART1_TX_Pin GPIO_PIN_9
#define CN5_6_USART1_TX_GPIO_Port GPIOA
#define CN5_34_USART1_RX_Pin GPIO_PIN_10
#define CN5_34_USART1_RX_GPIO_Port GPIOA
#define CN5_33_LED_Pin GPIO_PIN_6
#define CN5_33_LED_GPIO_Port GPIOB
#define CN5_4_I2C_SCL_Pin GPIO_PIN_7
#define CN5_4_I2C_SCL_GPIO_Port GPIOB
#define CN5_14_I2C_SDA_Pin GPIO_PIN_14
#define CN5_14_I2C_SDA_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
