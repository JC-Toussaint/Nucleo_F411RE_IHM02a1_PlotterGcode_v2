/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


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

/**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1 = PWM CH1
 */
void BSP_ServoMotorInit(void);
void BSP_ServoMotorOn(void);
void BSP_ServoMotorOff(void);
void BSP_ServoMotorSetAngle(uint8_t angle);

void plotterProcess(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define L6470_nBUSY_SYNC_Pin GPIO_PIN_0
#define L6470_nBUSY_SYNC_GPIO_Port GPIOC
#define L6470_nFLAG_Pin GPIO_PIN_1
#define L6470_nFLAG_GPIO_Port GPIOC
#define GPIO_EXTI_TRAVEL_STOP_X_Pin GPIO_PIN_2
#define GPIO_EXTI_TRAVEL_STOP_X_GPIO_Port GPIOC
#define GPIO_EXTI_TRAVEL_STOP_X_EXTI_IRQn EXTI2_IRQn
#define GPIO_EXTI_TRAVEL_STOP_Y_Pin GPIO_PIN_3
#define GPIO_EXTI_TRAVEL_STOP_Y_GPIO_Port GPIOC
#define GPIO_EXTI_TRAVEL_STOP_Y_EXTI_IRQn EXTI3_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define L6470_nCS_OP0_Pin GPIO_PIN_4
#define L6470_nCS_OP0_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define L6470_nCS_OP1_Pin GPIO_PIN_10
#define L6470_nCS_OP1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define L6470_nCS_OP3_Pin GPIO_PIN_4
#define L6470_nCS_OP3_GPIO_Port GPIOB
#define L6470_nSTBY_nRST_Pin GPIO_PIN_5
#define L6470_nSTBY_nRST_GPIO_Port GPIOB
#define L6470_nCS_OP2_Pin GPIO_PIN_6
#define L6470_nCS_OP2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
