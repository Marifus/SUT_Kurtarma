/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp180_for_stm32_hal.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */


float CalculateAltitude(const float* temp, const uint32_t* press, const uint32_t* ground_press) //irtifa hesaplayan fonksiyon
{
	if(*press == 0) return 0; // Sıfıra bölmemek için koruma.

	float alt = (powf((*ground_press / (float)*press), 0.190263f) - 1.0f) * (*temp + 273.15f) / 0.0065f;
	return alt;
}

void DeployDrogueParachute()
{
	//İlk paraşütü açan fonksiyon
	HAL_GPIO_WritePin(DROGUE_PARACHUTE_GPIO_Port, DROGUE_PARACHUTE_Pin, 1);
}

void DeployMainParachute()
{
	//İkinci paraşütü açan fonksiyon
	HAL_GPIO_WritePin(MAIN_PARACHUTE_GPIO_Port, MAIN_PARACHUTE_Pin, 1);
}

void DetermineGroundPressure(uint32_t* ground_press, uint32_t counter) //Referans basıncını almak için fonksiyon
{
	uint32_t press = 0;
	uint32_t sum = 0;
	float temp = 0.0f; //temp değerini kullanmıyoruz ama UpdateSensorData() fonksiyonunu nullptr ile çalışacak şekilde güncellemeye üşendim.

	for(int i=0; i<counter; i++)
	{
		UpdateSensorData(&temp, &press);
		sum += press;
		HAL_Delay(15); //Sensör verisi 13.5 ms'de bir güncelleniyor ve bunun için bekliyoruz.
	}

	*ground_press = (uint32_t)sum / counter;
}

float GetDerivative(const float* current_value, const uint32_t* current_time_ms, const float* prev_value, const uint32_t* prev_time_ms) //İRtifadan hıza, hızdan ivmeye geçmek için türev hesaplama fonksiyonu
{
	float delta_time = (*current_time_ms - *prev_time_ms) / 1000.0f; //iki sensör verisinin alındığı zaman arasındaki fark
	float delta_value = *current_value - *prev_value; //iki sensör verisi arasındaki fark

	if (delta_time <= 0.0) return 0.0f; // Sıfıra bölmemek veya delta_time'dan kaynaklanan bir hatayla negatif hız değeri almamak için koruma..

	return delta_value/delta_time;
}

float LowPassFilter(const float* raw, const float* prev, float lpf_coef) //Filtre fonksiyonu
{
	return (lpf_coef*(*raw) + (1-lpf_coef)*(*prev));
}

void UpdateSensorData(float* temp, uint32_t* press) //sensör verilerine göre değişkenleri güncelleme fonksiyonu
{
	*temp = BMP180_GetTemperature();
	*press = BMP180_GetPressure();
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char msg[128];

float acceleration = 0.0f;
float altitude = 0.0f;
float raw_altitude = 0.0f;
float temperature = 0.0f;
float velocity = 0.0f;
float previous_altitude = 0.0f;
float previous_velocity = 0.0f;

uint32_t current_time_ms = 0;
uint32_t ground_pressure = 0;
uint32_t pressure = 0;
uint32_t previous_ms = 0;

uint16_t counter = 0;

uint8_t len = 0;

rocket_status current_status = IDLE;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  BMP180_Init(&hi2c1);
  BMP180_SetOversampling(BMP180_HIGH);
  BMP180_UpdateCalibrationData();

  HAL_Delay(100); //Sensörün kalibre olması için bekliyoruz.
  DetermineGroundPressure(&ground_pressure, 20); //Referans basıncını hesaplamak için 20 ölçümün ortalamasını alıyoruz.

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
