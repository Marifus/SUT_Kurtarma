/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define APOGEE_ALTITUDE 8000.0
#define DETERMINED_ALTITUDE 2000.0
#define MAIN_DEPLOY_ALTITUDE 2000.0

#define DATA_BUFFER_SIZE 36
#define BOOST_DETECTED_BIT 0
#define BURNOUT_DETECTED_BIT 1
#define APOGEE_DETECTED_BIT 2
#define NOSE_DOWN_DETECTED_BIT 3
#define EMRE_DETECTED_BIT 4
#define DROGUE_PARACHUTE_DEPLOYED_BIT 5
#define BELOW_DETERMINED_ALTITUDE_BIT 6
#define MAIN_PARACHUTE_DEPLOYED_BIT 7
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

char msg[256];

float accel_x = 0.0f;
float accel_y = 0.0f;
float accel_z = 0.0f;
float altitude = 0.0f;
float euler_x = 0.0f;
float euler_y = 0.0f;
float euler_z = 0.0f;
float pressure = 0.0f;
float previous_accel_x = 0.0f;
float previous_accel_y = 0.0f;
float previous_accel_z = 0.0f;
float previous_altitude = 0.0f;
float previous_euler_x = 0.0f;
float previous_euler_y = 0.0f;
float previous_euler_z = 0.0f;
float previous_pressure = 0.0f;


uint32_t current_time_ms = 0;
uint32_t prev_time_ms = 0;

uint32_t ground_pressure = 0;

uint16_t counter = 0;

uint8_t len = 0;

rocket_status current_status = IDLE;

Sensor_Data current_data;

volatile uint8_t SIT_Task_Active = 0;
volatile uint8_t SUT_Task_Active = 0;
volatile uint16_t Task_Status_Bits = 0x0000;


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UARTYazdirma */
osThreadId_t UARTYazdirmaHandle;
const osThreadAttr_t UARTYazdirma_attributes = {
  .name = "UARTYazdirma",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorVeriCekme */
osThreadId_t SensorVeriCekmeHandle;
const osThreadAttr_t SensorVeriCekme_attributes = {
  .name = "SensorVeriCekme",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Algoritma */
osThreadId_t AlgoritmaHandle;
const osThreadAttr_t Algoritma_attributes = {
  .name = "Algoritma",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for UARTDinleme */
osThreadId_t UARTDinlemeHandle;
const osThreadAttr_t UARTDinleme_attributes = {
  .name = "UARTDinleme",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorDataMutex */
osMutexId_t SensorDataMutexHandle;
const osMutexAttr_t SensorDataMutex_attributes = {
  .name = "SensorDataMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


uint8_t CheckSum(uint8_t* rx_buffer, uint16_t rx_length, uint16_t header_index)
{
	uint8_t sum = 0;

	if (rx_length < header_index+DATA_BUFFER_SIZE) return 0;

	for(uint16_t i=header_index; i<header_index+33; i++)
	{
		sum = (uint8_t)(sum+rx_buffer[i]);
	}

	return sum;
}


void DeployDrogueParachute()
{
	//İlk paraşütü açan fonksiyon
	HAL_GPIO_WritePin(DROGUE_PARACHUTE_GPIO_Port, DROGUE_PARACHUTE_Pin, 1);
	Task_Status_Bits = (Task_Status_Bits | (1<<DROGUE_PARACHUTE_DEPLOYED_BIT));
}

void DeployMainParachute()
{
	//İkinci paraşütü açan fonksiyon
	HAL_GPIO_WritePin(MAIN_PARACHUTE_GPIO_Port, MAIN_PARACHUTE_Pin, 1);
	Task_Status_Bits = (Task_Status_Bits | (1<<MAIN_PARACHUTE_DEPLOYED_BIT));
}

float GetDerivative(const float* current_value, const uint32_t* current_time_ms, const float* prev_value, const uint32_t* prev_time_ms)
{
	float delta_time = (*current_time_ms - *prev_time_ms) / 1000.0f;
	float delta_value = *current_value - *prev_value;

	if (delta_time <= 0.0) return 0.0f;

	return delta_value/delta_time;
}

float GetIntegralAccel(const float* current_value, const uint32_t* current_time_ms, const uint32_t* prev_time_ms)
{
	static float integral = 0;

	float delta_time = (*current_time_ms - *prev_time_ms) / 1000.0f;
	integral += *current_value * delta_time;

	return integral;
}

float LowPassFilter(const float* raw, const float* prev, float lpf_coef)
{
	return (lpf_coef*(*raw) + (1-lpf_coef)*(*prev));
}

void SUTDataRead(uint8_t* rx_buffer, uint16_t rx_length, uint16_t header_index)
{
	if(rx_length-header_index < DATA_BUFFER_SIZE) return;

	if(rx_buffer[header_index+34] == 0x0D && rx_buffer[header_index+35] == 0x0A)
	{
		if(CheckSum(rx_buffer, rx_length, header_index) == rx_buffer[header_index+33])
		{
			FLOAT32_UINT8_CONVERTER converter;

			osMutexAcquire(SensorDataMutexHandle, osWaitForever);

			converter.array[0] = rx_buffer[header_index+1];
			converter.array[1] = rx_buffer[header_index+2];
			converter.array[2] = rx_buffer[header_index+3];
			converter.array[3] = rx_buffer[header_index+4];

			current_data.irtifa = converter.data_f32;

			converter.array[0] = rx_buffer[header_index+5];
			converter.array[1] = rx_buffer[header_index+6];
			converter.array[2] = rx_buffer[header_index+7];
			converter.array[3] = rx_buffer[header_index+8];

			current_data.basinc = converter.data_f32;

			converter.array[0] = rx_buffer[header_index+9];
			converter.array[1] = rx_buffer[header_index+10];
			converter.array[2] = rx_buffer[header_index+11];
			converter.array[3] = rx_buffer[header_index+12];

			current_data.ivme_x = converter.data_f32;

			converter.array[0] = rx_buffer[header_index+13];
			converter.array[1] = rx_buffer[header_index+14];
			converter.array[2] = rx_buffer[header_index+15];
			converter.array[3] = rx_buffer[header_index+16];

			current_data.ivme_y = converter.data_f32;

			converter.array[0] = rx_buffer[header_index+17];
			converter.array[1] = rx_buffer[header_index+18];
			converter.array[2] = rx_buffer[header_index+19];
			converter.array[3] = rx_buffer[header_index+20];

			current_data.ivme_z = converter.data_f32;

			converter.array[0] = rx_buffer[header_index+21];
			converter.array[1] = rx_buffer[header_index+22];
			converter.array[2] = rx_buffer[header_index+23];
			converter.array[3] = rx_buffer[header_index+24];

			current_data.aci_x = converter.data_f32;

			converter.array[0] = rx_buffer[header_index+25];
			converter.array[1] = rx_buffer[header_index+26];
			converter.array[2] = rx_buffer[header_index+27];
			converter.array[3] = rx_buffer[header_index+28];

			current_data.aci_y = converter.data_f32;

			converter.array[0] = rx_buffer[header_index+29];
			converter.array[1] = rx_buffer[header_index+30];
			converter.array[2] = rx_buffer[header_index+31];
			converter.array[3] = rx_buffer[header_index+32];

			current_data.aci_z = converter.data_f32;

			osThreadFlagsSet(SensorVeriCekmeHandle, 0x0001);
			osMutexRelease(SensorDataMutexHandle);
		}
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void TransferUARTData(void *argument);
void PullSensorData(void *argument);
void AlgorithmSwitch(void *argument);
void ReceiveUARTData(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of SensorDataMutex */
  SensorDataMutexHandle = osMutexNew(&SensorDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UARTYazdirma */
  UARTYazdirmaHandle = osThreadNew(TransferUARTData, NULL, &UARTYazdirma_attributes);

  /* creation of SensorVeriCekme */
  SensorVeriCekmeHandle = osThreadNew(PullSensorData, NULL, &SensorVeriCekme_attributes);

  /* creation of Algoritma */
  AlgoritmaHandle = osThreadNew(AlgorithmSwitch, NULL, &Algoritma_attributes);

  /* creation of UARTDinleme */
  UARTDinlemeHandle = osThreadNew(ReceiveUARTData, NULL, &UARTDinleme_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TransferUARTData */
/**
* @brief Function implementing the UARTYazdirma thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TransferUARTData */
void TransferUARTData(void *argument)
{
	uint32_t next_tick = osKernelGetTickCount();
	uint8_t tx_buffer[6];

  /* USER CODE BEGIN TransferUARTData */
  /* Infinite loop */
  for(;;)
  {
	next_tick += 100;

	  if(SUT_Task_Active)
	  {
		  tx_buffer[0] = 0xAA;
		  tx_buffer[1] = (uint8_t)(Task_Status_Bits & 0xFF);
		  tx_buffer[2] = (uint8_t)((Task_Status_Bits >> 8) & 0xFF);
		  tx_buffer[3] = CheckSum(tx_buffer, 3, 0);
		  tx_buffer[4] = 0x0D;
		  tx_buffer[5] = 0x0A;

		  HAL_UART_Transmit_DMA(&huart2, tx_buffer, 6);
	  }

    osDelayUntil(next_tick);
  }
  /* USER CODE END TransferUARTData */
}

/* USER CODE BEGIN Header_PullSensorData */
/**
* @brief Function implementing the SensorVeriCekme thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PullSensorData */
void PullSensorData(void *argument)
{
  /* USER CODE BEGIN PullSensorData */
  /* Infinite loop */
  for(;;)
  {

	  osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

	  osMutexAcquire(SensorDataMutexHandle, osWaitForever);

	  accel_x = LowPassFilter(&current_data.ivme_x, &previous_accel_x, 0.9);
	  accel_y = LowPassFilter(&current_data.ivme_y, &previous_accel_y, 0.9);
	  accel_z = LowPassFilter(&current_data.ivme_z, &previous_accel_z, 0.9);
	  altitude = LowPassFilter(&current_data.irtifa, &previous_altitude, 0.9);
	  euler_x = LowPassFilter(&current_data.aci_x, &previous_euler_x, 0.9);
	  euler_y = LowPassFilter(&current_data.aci_y, &previous_euler_y, 0.9);
	  euler_z = LowPassFilter(&current_data.aci_z, &previous_euler_z, 0.9);
	  pressure = LowPassFilter(&current_data.basinc, &previous_pressure, 0.9);



	  previous_accel_x = accel_x;
	  previous_accel_y = accel_y;
	  previous_accel_z = accel_z;
	  previous_altitude = altitude;
	  previous_euler_x = euler_x;
	  previous_euler_y = euler_y;
	  previous_euler_z = euler_z;
	  previous_pressure = pressure;

	  osMutexRelease(SensorDataMutexHandle);

  }
  /* USER CODE END PullSensorData */
}

/* USER CODE BEGIN Header_AlgorithmSwitch */
/**
* @brief Function implementing the Algoritma thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AlgorithmSwitch */
void AlgorithmSwitch(void *argument)
{
  /* USER CODE BEGIN AlgorithmSwitch */
  /* Infinite loop */
  for(;;)
  {

	  osMutexAcquire(SensorDataMutexHandle, 0);

	  switch (current_status)
	  {
	  	  case IDLE:
	  		  if(altitude > 50.0f && accel_z > 1.0f)
	  		  {
	  			  Task_Status_Bits = (Task_Status_Bits | (1<<BOOST_DETECTED_BIT));
	  			  current_status = BOOST;
	  		  }
	  		  else break;

	  	  case BOOST:
	  		  if(accel_z < -9.6f)
	  		  {
	  			  counter += 1;
	  			  if (counter < 5) break;
	  			  else
	  			  {
	  				  counter = 0;
	  				  Task_Status_Bits = (Task_Status_Bits | (1<<BURNOUT_DETECTED_BIT));
	  				  current_status = BURNOUT;
	  			  }
	  		  }

	  		  else
	  		  {
	  			  counter = 0;
	  			  break;
	  		  }

	  	  case BURNOUT:
	  		  if(altitude > APOGEE_ALTITUDE)
	  		  {
	  			  counter += 1;
	  			  if (counter < 5) break;
	  			  else
	  			  {
	  				  counter = 0;
	  				  Task_Status_Bits = (Task_Status_Bits | (1<<APOGEE_DETECTED_BIT));
	  				  current_status = APOGEE;
	  			  }
	  		  }

	  		  else
	  		  {
	  			  counter = 0;
	  			  break;
	  		  }

	  	  case APOGEE:
	  		  if (euler_x < -90 || euler_x > 90 || euler_y < -90 || euler_y > 90)
	  		  {
	  			  counter += 1;
	  			  if (counter < 5) break;
	  			  else
	  			  {
	  				  counter = 0;
	  				Task_Status_Bits = (Task_Status_Bits | (1<<NOSE_DOWN_DETECTED_BIT));
	  				  current_status = NOSE_DOWN;
	  			  }
	  		  }

	  		  else
	  		  {
	  			  counter = 0;
	  			  break;
	  		  }

	  	  case NOSE_DOWN:
	  		  if (previous_altitude > altitude)
	  		  {
	  			  counter += 1;
	  			  if (counter < 5) break;
	  			  else
	  			  {
	  				  counter = 0;
	  				  DeployDrogueParachute();
	  				  current_status = DROGUE_DESCENT;
	  			  }
	  		  }

	  		  else
	  		  {
	  			  counter = 0;
	  			  break;
	  		  }

	  	  case DROGUE_DESCENT:
	  		  if (altitude < DETERMINED_ALTITUDE)
	  		  {
	  			  counter += 1;
	  			  if (counter < 5) break;
	  			  else
	  			  {
	  				  counter = 0;
	  				Task_Status_Bits = (Task_Status_Bits | (1<<BELOW_DETERMINED_ALTITUDE_BIT));
	  				  current_status = BELOW_DETERMINED_ALTITUDE;
	  			  }
	  		  }

	  		  else
	  		  {
	  			  counter = 0;
	  			  break;
	  		  }

	  	  case BELOW_DETERMINED_ALTITUDE:
	  		  if (altitude < MAIN_DEPLOY_ALTITUDE)
	  		  {
	  			  counter += 1;
	  			  if (counter < 3) break;
	  			  else
	  			  {
	  				  counter = 0;
	  				  DeployMainParachute();
	  				  current_status = MAIN_DESCENT;
	  			  }
	  		  }

	  		  else
	  		  {
	  			  counter = 0;
	  			  break;
	  		  }

	  	  case MAIN_DESCENT:
	  		  if (accel_z < 0.5 && altitude < 100.0f)
	  		  {
	  			  counter += 1;

	  			  if (counter < 5) break;
	  			  else
	  			  {
	  				  counter = 0;
	  				  current_status = LANDED;
	  			  }
	  		  }

	  		  break;

	  	  case LANDED:
		//buzzer togglepin

	  	  default:

	  }

	  osMutexRelease(SensorDataMutexHandle);

    osDelay(10);
  }
  /* USER CODE END AlgorithmSwitch */
}

/* USER CODE BEGIN Header_ReceiveUARTData */
/**
* @brief Function implementing the UARTDinleme thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReceiveUARTData */
void ReceiveUARTData(void *argument)
{
  /* USER CODE BEGIN ReceiveUARTData */
	uint8_t rx_buffer[100];
	uint16_t rx_length;
	uint16_t header_index = 0;

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, sizeof(rx_buffer));
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
	  rx_length = sizeof(rx_buffer) - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

	  for (int i = 0; i<rx_length; i++)
	  {
		  if(rx_buffer[i] == 0xAA || rx_buffer[i] == 0xAB)
		  {
			  header_index = i;
			  break;
		  }

	  }


	  if (rx_length >= header_index+5 && rx_buffer[header_index] == 0xAA)
	  {
		  if (rx_buffer[header_index+3] == 0x0D && rx_buffer[header_index+4] == 0x0A)
		  {
			  if (rx_buffer[header_index+1] == 0x20 && rx_buffer[header_index+2] == 0x8C)
			  {
				  osDelay(1000);
				  SIT_Task_Active = 1;
				  SUT_Task_Active = 0;
			  }

			  else if (rx_buffer[header_index+1] == 0x22 && rx_buffer[header_index+2] == 0x8E)
			  {
				  osDelay(1000);
				  SIT_Task_Active = 0;
				  SUT_Task_Active = 1;
			  }

			  else if (rx_buffer[header_index+1] == 0x24 && rx_buffer[header_index+2] == 0x90)
			  {
				  SIT_Task_Active = 0;
				  SUT_Task_Active = 0;
			  }
		  }
	  }

		  else if ((SUT_Task_Active == 1) && rx_buffer[header_index] == 0xAB)
		  {
			  SUTDataRead(rx_buffer, rx_length, header_index);
		  }
	  }

  /* USER CODE END ReceiveUARTData */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

