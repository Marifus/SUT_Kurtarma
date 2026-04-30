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
typedef enum {
	IDLE,
	BOOST,
	BURNOUT,
	APOGEE,
	NOSE_DOWN,
	DROGUE_DESCENT,
	BELOW_DETERMINED_ALTITUDE,
	MAIN_DESCENT,
	LANDED
} rocket_status;

typedef struct {
	float irtifa;
	float basinc;
	float ivme_x;
	float ivme_y;
	float ivme_z;
	float aci_x;
	float aci_y;
	float aci_z;
} Sensor_Data;

typedef union {
    float data_f32;
    uint8_t array[4];
} FLOAT32_UINT8_CONVERTER;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MAIN_PARACHUTE_Pin GPIO_PIN_4
#define MAIN_PARACHUTE_GPIO_Port GPIOB
#define DROGUE_PARACHUTE_Pin GPIO_PIN_5
#define DROGUE_PARACHUTE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
