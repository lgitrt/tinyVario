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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define HALT_WITH_ERROR(err) do { g_system_error = (err); Error_Handler(); } while(0);

typedef enum { STATE_OFF, STATE_ACTIVE, STATE_CHARGING } SystemState_t;
extern SystemState_t current_state;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef enum {
    ERR_UNKNOWN = 0,
    ERR_SYS_CLOCK,
    ERR_ADC_INIT,
    ERR_I2C_INIT,
    ERR_TIM2_INIT,
    ERR_TIM21_INIT,
    ERR_BARO_SPL06,
    ERR_IMU_LSM6DS3,
    ERR_EEPROM_STORAGE
} SystemError_t;
extern volatile SystemError_t g_system_error;
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
#define ButtonIn_Pin GPIO_PIN_0
#define ButtonIn_GPIO_Port GPIOA
#define BATvoltage_Pin GPIO_PIN_1
#define BATvoltage_GPIO_Port GPIOA
#define USBsens_Pin GPIO_PIN_2
#define USBsens_GPIO_Port GPIOA
#define CHstat_Pin GPIO_PIN_4
#define CHstat_GPIO_Port GPIOA
#define BuzzerPWM_Pin GPIO_PIN_5
#define BuzzerPWM_GPIO_Port GPIOA
#define BATled1_Pin GPIO_PIN_6
#define BATled1_GPIO_Port GPIOA
#define BATled2_Pin GPIO_PIN_7
#define BATled2_GPIO_Port GPIOA
#define acc_INT_Pin GPIO_PIN_1
#define acc_INT_GPIO_Port GPIOB
#define acc_INT_EXTI_IRQn EXTI0_1_IRQn
#define BATled3_Pin GPIO_PIN_8
#define BATled3_GPIO_Port GPIOA
#define BUZ_EN2_Pin GPIO_PIN_15
#define BUZ_EN2_GPIO_Port GPIOA
#define BUZ_EN1_Pin GPIO_PIN_3
#define BUZ_EN1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
