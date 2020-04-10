/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN PFP */

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
  HAL_Init();
  SystemClock_Config();

	// Enable TIM2, TIM3, GPIOC peripherals in RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set conditions & intialization for LED 8&9 pins
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr);
	
	// Sets conditions & initialization for LED 6&7
	GPIO_InitTypeDef initStr1 = {GPIO_PIN_6 | GPIO_PIN_7,
	GPIO_MODE_AF_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr1);
	
	GPIOC->ODR |= (1 << 8);
	
	// Sets AF1 for PC6&7 for Ch1&2
	GPIOC->AFR[0] &= (0x00 << GPIO_AFRL_AFRL6_Pos)|(0x00 << GPIO_AFRL_AFRL7_Pos);
	
	// fTARGET = 4Hz
	TIM2->PSC = 0x1F3F; // PSC = 7999
	TIM2->ARR = 0xFA; // ARR = 250
	TIM2->DIER |= TIM_DIER_UIE; // Enable UEV
	TIM2->CR1 |= TIM_CR1_CEN;	// ENABLE TIM2
	
	// fTARGET = 800Hz
	TIM3->PSC = 79;
	TIM3->ARR = 125; 
	TIM3->CCMR1 &= ~(0x3 << TIM_CCMR1_CC1S_Pos) | ~(0x3 << TIM_CCMR1_CC1S_Pos); // CCS1/2 - > Output (00)
	TIM3->CCMR1 |= (0x7 << TIM_CCMR1_OC1M_Pos) | (0x6 << TIM_CCMR1_OC2M_Pos); // OC1M (111 - PWM MODE 2), OC2M (110 - PWM MODE 1)
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; // Output compare pre-enable, enabled for ch1&2
	TIM3->CCER	|= TIM_CCER_CC1E | TIM_CCER_CC2E; // Output enabled for ch1&2
	TIM3->CCR1 = 1; // 20% ARR
	TIM3->CCR2 = 25; // 20% ARR
	TIM3->CR1	|= TIM_CR1_CEN;

	// Configure NVIC
	NVIC_EnableIRQ(TIM2_IRQn);	// Enable interrupt
	NVIC_SetPriority(TIM2_IRQn,1);	// Set priority

uint16_t counter = 0;
uint8_t counting_up = 1;

  while (1)
  {
		if (counting_up){
			counter++;
			if (counter == 125) {
				counting_up = 0;
			}
		}
		else {
			counter--;
			if (counter == 0){
				counting_up = 1;
			}
		}
		HAL_Delay(3);
		TIM3->CCR1 = counter;
		TIM3->CCR2 = counter;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
