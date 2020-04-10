

/* Includes ------------------------------------------------------------------*/
#include "main.h"
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	// Clock enable: GPIOB/C, I2C2
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// PB11/13 - AF1, OD
	// PB14/PC0 - OUTPUT, PP
	GPIOB->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_0; // PB11/13 - AF, PF14 - OUT
	GPIOC->MODER |= GPIO_MODER_MODER0_0;	// PC0 - OUT
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_13;		// PB11/13 - OD
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);	// PB14 - PP
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_0);	// PC0 - PP
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_0|GPIO_PUPDR_PUPDR13_0;	// PB11/13 PU
	GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFRH3_Pos)|(0x5 << GPIO_AFRH_AFRH5_Pos);	// PB11 - AF1, PB13 - AF5
	GPIOB->ODR |= GPIO_ODR_14;	// Set PB14 HIGH
	GPIOC->ODR |= GPIO_ODR_0;	// Set PC0 HIGH
	
	// Set conditions & intialization for LEDs
	GPIO_InitTypeDef initStr2 = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr2);

	// I2C2 INITIALIZATION
	// 100kHz config for our 8 MHz processor
	I2C2->TIMINGR = (1 << I2C_TIMINGR_PRESC_Pos)|(0x13 << I2C_TIMINGR_SCLL_Pos)|(0xF << I2C_TIMINGR_SCLH_Pos)|(0x2 << I2C_TIMINGR_SDADEL_Pos)|(0x4 << I2C_TIMINGR_SCLDEL_Pos);
	I2C2->CR1 |= I2C_CR1_PE;
	
	// Request a WRITE to SLAVE @ 0x6B
	I2C2->CR2 = (1 << 16)|(0x6B << 1)|I2C_CR2_START;	// Send 1 Byte to slave address 0x6b
	
	// Poll TXIS & NACKF
	while(((I2C2->ISR & I2C_ISR_TXIS)==0)&&((I2C2->ISR & I2C_ISR_NACKF)==0));
	
	// If request not acknowledged...
	if((I2C2->ISR & I2C_ISR_NACKF)==I2C_ISR_NACKF){
				GPIOC->ODR = GPIO_ODR_6;	// Red
			}
	
	// If request acknowledged... write WHO_AM_I to TXDR
	if ((I2C2->ISR & I2C_ISR_TXIS)==I2C_ISR_TXIS){
			GPIOC->ODR = GPIO_ODR_9; // Green
			I2C2->TXDR = 0x0F;
		}

	// Poll TC & NACK
	while(((I2C2->ISR & I2C_ISR_TC)==0)&&((I2C2->ISR & I2C_ISR_NACKF)==0));
	
	if((I2C2->ISR & I2C_ISR_NACKF)){
			GPIOC->ODR = GPIO_ODR_6|GPIO_ODR_9;	// Red + Green
		}
	
	// If all data transmitted
		I2C2->CR2 |= I2C_CR2_STOP;

			// Request a READ from SLAVE @ 0x6B
			GPIOC->ODR = GPIO_ODR_7|GPIO_ODR_9;	// Green + Blue
			I2C2->CR2 = (1 << 16)|(0x6B << 1)|I2C_CR2_RD_WRN| I2C_CR2_START;
	
	// Poll TXIS & NACKF
	while(((I2C2->ISR & I2C_ISR_RXNE)==0)&&((I2C2->ISR & I2C_ISR_NACKF)==0));
	
	// If request not acknowledged...
	if((I2C2->ISR & I2C_ISR_NACKF)){
		GPIOC->ODR = GPIO_ODR_6|GPIO_ODR_7|GPIO_ODR_9;	// Red + Green + Blue
	}
	
	// If there's something to be received...
	if((I2C2->ISR & I2C_ISR_RXNE)){
		while((I2C2->ISR & I2C_ISR_TC)==0);
		if(I2C2->RXDR == 0xD4){
			GPIOC->ODR = GPIO_ODR_8|GPIO_ODR_7|GPIO_ODR_9;	// Orange + Green + Blue
		}
		else{
			GPIOC->ODR = GPIO_ODR_6|GPIO_ODR_8;	// Green + Orange
		}
	}
  while (1)
  {
		
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

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
