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
int left=4;
int right=4;
int storeLeft=4;
int storeRight=4;
char data[5]= "L4R4:";
char dataChar;
int i = 0;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void processBTData(void);
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
  /* USER CODE BEGIN 2 */
		
		RCC->APB1ENR |= RCC_APB1ENR_USART4EN;//enable the clock for USART4'
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;//enable GPIOC clock
		
		//GPIOC->MODER |= (1 << 9) | (1 << 11);
		//GPIOC->MODER |= (1 << 21) | (1 << 23);
		GPIOC->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
		//enable alternate functions for PC4 and PC5 as TX and RX respectively
		//GPIOC->AFR[0] |= ((1 << 16) | (1 << 20));
		//enable alternate functions for PC10 and PC11 as TX and RX respectively
		//GPIOC->AFR[1] &= ~((1 << 8) | (1 << 12));
		//set the baud rate
		//USART3->BRR = HAL_RCC_GetHCLKFreq()/9600;
		USART4->BRR = HAL_RCC_GetHCLKFreq()/9600;
		//Enable the transmitter and receiver
		//USART3->CR1 |= ((1<<5) | (1<<3) | (1 << 2));
		//USART3->CR1 |= (1);
		USART4->CR1 |= ((1<<5) | (1<<3) | (1 << 2));
		USART4->CR1 |= (1);
		//set interupt
		
		
		GPIOC->MODER |= (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18);
		GPIOC->OTYPER &= ~((1 << 7) | (1 << 6) | (1 << 8) | (1 << 9));
		GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18)); 
		GPIOC->PUPDR &= ~((1 << 12)|(1 << 13)|(1 << 14)|(1 << 15)|(1 << 16)|(1 << 17)|(3 << 18));
		GPIOC->ODR &= ~((1 << 6)|(1 << 7)|(1 << 8)|(1 << 9)); // set all other led's to low.
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(left>4){
			GPIOC->ODR &= ~((1 << 8)|(1 << 9));
			GPIOC->ODR |= (1 << 9);
		}
		else if(left==4){
			GPIOC->ODR &= ~((1 << 8)|(1 << 9));
		}
		else if(left<4){
			GPIOC->ODR &= ~((1 << 8)|(1 << 9));
			GPIOC->ODR |= (1 << 8);
		}
		
		if(right>4){
			GPIOC->ODR &= ~((1 << 6) | (1 << 7));
			GPIOC->ODR |= (1 << 7);
		}
		else if(right==4){
			GPIOC->ODR &= ~((1 << 6) | (1 << 7));
		}
		else if(right<4){
			GPIOC->ODR &= ~((1 << 6) | (1 << 7));
			GPIOC->ODR |= (1 << 6);
		}
		
			processBTData();	
	}
  /* USER CODE END 3 */
}

/*void USART3_4_IRQHandler(){
		if((USART3->ISR & USART_ISR_RXNE)){
				data = USART3->RDR;
				
		}
}*/

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
void processBTData(void){
		if ((USART4->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) //check if the ISR register is not empty
		{
			dataChar = (uint8_t)(USART4->RDR); // Receive data, clear flag
			data[i] = dataChar; //store char
			if(dataChar==':'){
				i=0;
				left = data[1]-48;
				right = data[3]-48;
			}
			else{
				i++;
			}
			
		}
}
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
