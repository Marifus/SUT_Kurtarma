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
#include "bmp180_for_stm32_hal.h"
#include "stdio.h"
#include "math.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
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
#define MAIN_DEPLOY_ALTITUDE 2000.0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern char msg[128];

extern float acceleration;
extern float altitude;
extern float raw_altitude;
extern float temperature;
extern float velocity;
extern float previous_altitude;
extern float previous_velocity;

extern uint32_t current_time_ms;
extern uint32_t ground_pressure;
extern uint32_t pressure;
extern uint32_t previous_ms;

extern uint16_t counter;

extern uint8_t len;

extern rocket_status current_status;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId YazdirmaHandle;
osThreadId SensorVeriCekmeHandle;
osThreadId AlgoritmaHandle;
osSemaphoreId SensorDataAvailableHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

extern float CalculateAltitude(const float* temp, const uint32_t* press, const uint32_t* ground_press);
extern void DeployDrogueParachute();
extern void DeployMainParachute();
extern void DetermineGroundPressure(uint32_t* ground_press, uint32_t counter);
extern float GetDerivative(const float* current_value, const uint32_t* current_time_ms, const float* prev_value, const uint32_t* prev_time_ms);
extern float LowPassFilter(const float* raw, const float* prev, float lpf_coef);
extern void UpdateSensorData(float* temp, uint32_t* press);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void PrintUART(void const * argument);
void PullSensorData(void const * argument);
void AlgorithmSwitch(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of SensorDataAvailable */
  osSemaphoreDef(SensorDataAvailable);
  SensorDataAvailableHandle = osSemaphoreCreate(osSemaphore(SensorDataAvailable), 1);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Yazdirma */
  osThreadDef(Yazdirma, PrintUART, osPriorityLow, 0, 256);
  YazdirmaHandle = osThreadCreate(osThread(Yazdirma), NULL);

  /* definition and creation of SensorVeriCekme */
  osThreadDef(SensorVeriCekme, PullSensorData, osPriorityHigh, 0, 256);
  SensorVeriCekmeHandle = osThreadCreate(osThread(SensorVeriCekme), NULL);

  /* definition and creation of Algoritma */
  osThreadDef(Algoritma, AlgorithmSwitch, osPriorityAboveNormal, 0, 256);
  AlgoritmaHandle = osThreadCreate(osThread(Algoritma), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_PrintUART */
/**
* @brief Function implementing the Yazdirma thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PrintUART */
void PrintUART(void const * argument)
{
  /* USER CODE BEGIN PrintUART */
  /* Infinite loop */
  for(;;)
  {
	  len = sprintf(msg, "Hiz: %.2f   |   Irtifa: %.2f   |   Basinc: %ld\n", velocity, altitude, pressure);
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
    osDelay(50);
  }
  /* USER CODE END PrintUART */
}

/* USER CODE BEGIN Header_PullSensorData */
/**
* @brief Function implementing the SensorVeriCekme thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PullSensorData */
void PullSensorData(void const * argument)
{
  /* USER CODE BEGIN PullSensorData */
  /* Infinite loop */
  for(;;)
  {

	  if (xSemaphoreTake(SensorDataAvailableHandle, 0) == pdTRUE)
	  {
		  UpdateSensorData(&temperature, &pressure); //sensör verisi ile değişkenleri güncelliyoruz.
		  current_time_ms = HAL_GetTick(); //değişkenlerin güncellendiği anı alıyoruz. Direkt UpdateSensorData() fonksiyonunda belirlemeyi düşündüm ama fonksiyonun esnek kalmasını istedim.
		  raw_altitude = CalculateAltitude(&temperature, &pressure, &ground_pressure); //ham irtifa verisi hesaplama.
		  altitude = LowPassFilter(&raw_altitude, &previous_altitude, 0.2); //irtifa verisini filtreden geçiriyoruz.
		  velocity = GetDerivative(&altitude, &current_time_ms, &previous_altitude, &previous_ms); //irtifanın türeviyle hız hesaplama.
		  acceleration = GetDerivative(&velocity, &current_time_ms, &previous_velocity, &previous_ms); //hızın türeviyle ivme hesaplama.

		  previous_altitude = altitude;
		  previous_ms = current_time_ms;
		  previous_velocity = velocity;

		  xSemaphoreGive(SensorDataAvailableHandle);
	  }


    osDelay(14);
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
void AlgorithmSwitch(void const * argument)
{
  /* USER CODE BEGIN AlgorithmSwitch */
  /* Infinite loop */
  for(;;)
  {


	  if (xSemaphoreTake(SensorDataAvailableHandle, 0) == pdTRUE)
	  {

		  switch (current_status)
		  {
		case IDLE: //roketin rampada olduğu an.
			if(altitude > 100.0f && velocity > 1.0f) //100 metre yükselmiş ve hız 1 m/s olmuşsa sonraki aşamaya geç.
				current_status = BOOST;
			else break;

		case BOOST:
			if(acceleration < -9.6f) // İvme yerçekimi ivmesine eşitlenirse roketin yakıtı bitmiş demektir (0.21f hata payı)
			{
				counter += 1;
				if (counter < 5) break;
				else
				{
					counter = 0;
					current_status = BURNOUT;
				}
			}

			else
			{
				counter = 0;
				break;
			}

		case BURNOUT:
			if(velocity < 0.0f) // Hız aşağı yönlüyse roket düşüyordur. Sürüklenme paraşütünü aç.
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
			if (altitude < MAIN_DEPLOY_ALTITUDE) // Roket ana paraşüt ateşlenmesi için belirlenen irtifanın altındaysa ana paraşütü aç.
			{
				counter += 1;
				if (counter < 5) break;
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
			if (!(velocity < -1.0f || velocity > 1.0f) && altitude < 100.0f) // Hız 0 ve irtifa 10'un altındaysa roket yere düşmüştür.
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

		  xSemaphoreGive(SensorDataAvailableHandle);
	  }


    osDelay(1);
  }
  /* USER CODE END AlgorithmSwitch */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
