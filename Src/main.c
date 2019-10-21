/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include <string.h>
	#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

volatile uint32_t dht_value[100];
volatile uint32_t dht_bit;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CHECK_BIT(var, pos) (((var) & (1UL << (pos))) != 0)
//#define SET_BIT(var, pos) ((var) |= (1UL << (pos)))
#define CLR_BIT(var, pos) (var &= ~(1UL << (pos)))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void local_delay(unsigned int t);
uint8_t reorder (uint8_t _in);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  	  HAL_TIM_Base_Start(&htim3);
	char DataChar[100];
	sprintf(DataChar,"\r\n DHT11\r\nUART1 for debug started on speed 115200\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	  HAL_GPIO_WritePin(DHT11_WRITE_GPIO_Port, DHT11_WRITE_Pin, GPIO_PIN_RESET);
	  HAL_Delay(20);
	  HAL_GPIO_WritePin(DHT11_WRITE_GPIO_Port, DHT11_WRITE_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	  uint8_t temp = 0;
	  uint8_t hugo = 0;
	  uint8_t   cs = 0;

	  HAL_GPIO_WritePin(DHT11_WRITE_GPIO_Port, DHT11_WRITE_Pin, GPIO_PIN_RESET);
	  HAL_Delay(20);
	  HAL_GPIO_WritePin(DHT11_WRITE_GPIO_Port, DHT11_WRITE_Pin, GPIO_PIN_SET);
	  dht_bit = 0;
	  HAL_Delay(500);

	  for (int i=2; i<42; i++) {
		sprintf(DataChar,"  %03d" , (int)dht_value[i]);
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	    if ( (dht_value[i] > 66) && (dht_value[i] < 88) ) {
			  sprintf(DataChar,"(0)");
			  HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	    }

		if ( (dht_value[i] > 115) && (dht_value[i] < 135) ) {
			  sprintf(DataChar,"(1)");
			  HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
			  if (i < 13)						hugo = hugo | (1UL << (i-2      ) );
			  else if ((i > 15) && (i < 30))	temp = temp | (1UL << (i-2-16   ) );
			  else if  (i > 30)					cs 	 =   cs | (1UL << (i-2-16-16) );
		}

		if ( (i>7) && ((i-1)%8 == 0) ) {
			sprintf(DataChar,"\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
		}
	  }

	  hugo = reorder(hugo);
	  temp = reorder(temp);
	  cs   = reorder(  cs);

	sprintf(DataChar,"hugo:%d; temp:%d;", (int)hugo, (int)temp );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	if (cs == hugo + temp) {
		sprintf(DataChar," cs->Ok." );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	} else {
		sprintf(DataChar," cs:%d;", (int)cs );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	}
	sprintf(DataChar,"\r\n \r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);


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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/***************************************************************/
uint8_t reorder (uint8_t _in) {
	uint8_t result = 0;
	for (int i=0; i<8; i++) {
		if (CHECK_BIT(_in,i)) {
			result |= (1UL << (7-i));
		}
	}
	return result;
}
/***************************************************************/

void local_delay(unsigned int t) {
	for (; t > 0; t--) {
		__asm("nop");
	}
}
/***************************************************************/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
