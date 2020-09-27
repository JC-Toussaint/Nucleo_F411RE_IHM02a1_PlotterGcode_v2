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
#ifndef __XNUCLEO_INTERFACE_H
#define __XNUCLEO_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @addtogroup X-NUCLEO-IHM02A1
  * @{
  */

/**
  * @addtogroup NUCLEO_Interface
  * @{
  */

/**
  * @addtogroup NUCLEO_Feature_Selection
  * @{
  */

#define NUCLEO_USE_SPI_1                //!< Uncomment to set SPI1 to iterface with the X-NUCLEO-IHM02A1.
//#define NUCLEO_USE_SPI_2                //!< Uncomment to set SPI2 to iterface with the X-NUCLEO-IHM02A1.
#if ((defined (NUCLEO_USE_SPI_1)) && (defined (NUCLEO_USE_SPI_2)))
  #error "Please select an option only!"
#elif ((!defined (NUCLEO_USE_SPI_1)) && (!defined (NUCLEO_USE_SPI_2)))
  #error "Please select an option!"
#endif

#define NUCLEO_USE_USER_LED     //!< Uncomment to enable the NUCLEO User LED feature.
#define NUCLEO_USE_USER_BUTTON  //!< Uncomment to enable the NUCLEO User Button feature.
//#define NUCLEO_USE_USART        //!< Uncomment to enable the NUCLEO USART feature.

#define USE_ST1S14_PGOOD        //!< Uncomment to use ADC for reading PGOOD of ST1S14

/**
  * @}
  */ /* End of NUCLEO_Feature_Selection */

/**
  * @defgroup   NUCLEO_Exported_Macros
  * @brief      NUCLEO Exported Macros.
  * @{
  */

#ifdef NUCLEO_USE_SPI_1                 //!< in this case two options about the SPI_SCK are possible
#define USE_PB3_AS_SCK                  //!< Uncomment to set PB3 as SPI SCK (SB34)
//#define USE_PA5_AS_SCK                  //!< Uncomment to set PA5 as SPI SCK (SB12)
#if ((defined (USE_PB3_AS_SCK)) && (defined (USE_PA5_AS_SCK)))
  #error "Please select an option only!"
#endif
#endif

/* Redefined the data structures to address SPIx */
#if defined (NUCLEO_USE_SPI_1)
  #define HSPI                                  hspi1   //!< The data structure for all further instances to SPI.
#elif defined (NUCLEO_USE_SPI_2)
  #define HSPI                                  hspi2
#endif

/* Redefined the data structures to address ADCx */
#define HADC                                    hadc1  //!< The data structure for all further instances to ADC.

/**
  * @brief  GPIO clock enable
  *
  * @param  OPx_GPIO_CLK_ENABLE RCC AHB peripheral clock register
  */
#define GPIO_CLK_ENABLE(OPx_GPIO_CLK_ENABLE)    (RCC->AHB1ENR |= (OPx_GPIO_CLK_ENABLE))

/* GPIO settings for the nSTBY_nRST pins of the L6470 */
#define L6470_nSTBY_nRST_GPIO_CLK_ENABLE        RCC_AHB1ENR_GPIOBEN   		//!< nSTBY_nRST, RCC AHB peripheral clock register
//#define L6470_nSTBY_nRST_GPIO_PORT              L6470_nSTBY_nRST_GPIO_Port  //!< nSTBY_nRST, GPIO Port
//#define L6470_nSTBY_nRST_GPIO_PIN               L6470_nSTBY_nRST_Pin        //!< nSTBY_nRST, GPIO Pin

/* GPIO settings for the nBUSY_SYNC pin of the L6470 */
#define L6470_nBUSY_SYNC_GPIO_CLK_ENABLE        RCC_AHB1ENR_GPIOCEN   		//!< nBUSY_SYNC, RCC AHB1 peripheral clock register
//#define L6470_nBUSY_SYNC_GPIO_PORT              L6470_nBUSY_SYNC_GPIO_Port  //!< nSTBY_nRST, GPIO Port
//#define L6470_nBUSY_SYNC_GPIO_PIN               L6470_nBUSY_SYNC_Pin        //!< nSTBY_nRST, GPIO Pin

#define L6470_nBUSY_SYNC_IRQn                   EXTI0_IRQn            		//!< nSTBY_nRST, IRQ number
#define L6470_nBUSY_SYNC_EXTI_IRQHandler        EXTI0_IRQHandler      		//!< nSTBY_nRST, IRQ handler name

/* GPIO settings for the nFLAG pin of the L6470 */
#define L6470_nFLAG_GPIO_CLK_ENABLE             RCC_AHB1ENR_GPIOCEN   		//!< nFLAG, RCC AHB1 peripheral clock register
//#define L6470_nFLAG_GPIO_PORT                   L6470_nFLAG_GPIO_Port       //!< nFLAG, GPIO Port
//#define L6470_nFLAG_GPIO_PIN                    L6470_nFLAG_Pin            	//!< nFLAG, GPIO Pin

#define L6470_nFLAG_IRQn                        EXTI1_IRQn            		//!< nFLAG, IRQ number
#define L6470_nFLAG_EXTI_IRQHandler             EXTI1_IRQHandler      		//!< nFLAG, IRQ handler name

/* GPIO settings for the options about the nCS pins of the L6470 */
#define L6470_nCS_OP0_GPIO_CLK_ENABLE           RCC_AHB1ENR_GPIOAEN   		//!< nCS Option 0, RCC AHB peripheral clock register
//#define L6470_nCS_OP0_GPIO_PORT                 L6470_nCS_OP0_GPIO_Port     //!< nCS Option 0, GPIO Port
//#define L6470_nCS_OP0_GPIO_PIN                  L6470_nCS_OP0_Pin           //!< nCS Option 0, GPIO Pin

#define L6470_nCS_OP1_GPIO_CLK_ENABLE           RCC_AHB1ENR_GPIOAEN   		//!< nCS Option 1, RCC AHB peripheral clock register
//#define L6470_nCS_OP1_GPIO_PORT                 L6470_nCS_OP1_GPIO_Port     //!< nCS Option 1, GPIO Port
//#define L6470_nCS_OP1_GPIO_PIN                  L6470_nCS_OP1_Pin           //!< nCS Option 1, GPIO Pin

#define L6470_nCS_OP2_GPIO_CLK_ENABLE           RCC_AHB1ENR_GPIOBEN   		//!< nCS Option 2, RCC AHB peripheral clock register
//#define L6470_nCS_OP2_GPIO_PORT                 L6470_nCS_OP2_GPIO_Port     //!< nCS Option 2, GPIO Port
//#define L6470_nCS_OP2_GPIO_PIN                  L6470_nCS_OP2_Pin           //!< nCS Option 2, GPIO Pin

#define L6470_nCS_OP3_GPIO_CLK_ENABLE           RCC_AHB1ENR_GPIOBEN   		//!< nCS Option 3, RCC AHB peripheral clock register
//#define L6470_nCS_OP3_GPIO_PORT                 L6470_nCS_OP3_GPIO_Port     //!< nCS Option 3, GPIO Port
//#define L6470_nCS_OP3_GPIO_PIN                  L6470_nCS_OP3_Pin           //!< nCS Option 3, GPIO Pin

/**
  * @}
  */ /* End of NUCLEO_Exported_Macros */

/**
  * @addtogroup NUCLEO_Exported_Types
  * @brief      NUCLEO Exported Types.
  * @{
  */

/**
  * @brief  The structure to store GPIO information about the NUCLEO board pin
  *         connected with the L6470 one.
  */
typedef struct {
  uint32_t gpio_clk_enable;   //!< RCC AHB peripheral clock register
  GPIO_TypeDef* port;         //!< GPIO Port
  uint16_t pin;               //!< GPIO Pin
} sL6470_GPIO;

/**
  * @}
  */ /* End of NUCLEO_Exported_Types */

/**
  * @addtogroup NUCLEO_Exported_Variables
  * @brief      NUCLEO Exported Variables.
  * @{
  */

extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;
extern sL6470_GPIO L6470_nSTBY_nRST_GPIO;
extern sL6470_GPIO L6470_nBUSY_SYNC_GPIO;
extern sL6470_GPIO L6470_nFLAG_GPIO;
extern sL6470_GPIO L6470_nCS_GPIO;
extern sL6470_GPIO L6470_nCS_GPIO_Array[];

/**
  * @}
  */ /* End of NUCLEO_Exported_Variables */

/**
  * @defgroup   NUCLEO_Exported_Functions
  * @{
  */

void NUCLEO_Board_Init(void);
void User_LED_Blinking(uint8_t repetitions, uint16_t period_ms);

/**
  * @}
  */ /* End of NUCLEO_Exported_Functions */

/**
  * @}
  */ /* End of NUCLEO_Interface */

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1 */

/**
  * @}
  */ /* End of BSP */

#ifdef __cplusplus
}
#endif

#endif /* __XNUCLEO_INTERFACE_H */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
