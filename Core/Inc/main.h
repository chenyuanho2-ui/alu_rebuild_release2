/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define btn_foot_Pin GPIO_PIN_4
#define btn_foot_GPIO_Port GPIOD
#define SPI2_CS_T_Pin GPIO_PIN_3
#define SPI2_CS_T_GPIO_Port GPIOD
#define SDIO_GND_Pin GPIO_PIN_14
#define SDIO_GND_GPIO_Port GPIOH
#define flag_485_Pin GPIO_PIN_7
#define flag_485_GPIO_Port GPIOB
#define SDIO_detect_Pin GPIO_PIN_13
#define SDIO_detect_GPIO_Port GPIOH
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define TIM1_PWM_Pin GPIO_PIN_11
#define TIM1_PWM_GPIO_Port GPIOA
#define Touch_RST_Pin GPIO_PIN_10
#define Touch_RST_GPIO_Port GPIOI
#define Touch_INT_Pin GPIO_PIN_11
#define Touch_INT_GPIO_Port GPIOI
#define Touch_IIC_SDA_Pin GPIO_PIN_7
#define Touch_IIC_SDA_GPIO_Port GPIOG
#define Touch_IIC_SCL_Pin GPIO_PIN_3
#define Touch_IIC_SCL_GPIO_Port GPIOG
#define LTDC_BackLight_Pin GPIO_PIN_6
#define LTDC_BackLight_GPIO_Port GPIOH
#define btn_1_Pin GPIO_PIN_12
#define btn_1_GPIO_Port GPIOB
#define btn_4_Pin GPIO_PIN_15
#define btn_4_GPIO_Port GPIOB
#define btn_2_Pin GPIO_PIN_13
#define btn_2_GPIO_Port GPIOB
#define btn_3_Pin GPIO_PIN_14
#define btn_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
