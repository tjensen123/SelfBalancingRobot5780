#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

void SystemClock_Config(void);
void GPIO_Init(void);
void GYRO_Init(void);
void SPI_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
	GPIO_Init();
	SPI_Init();
	GYRO_Init();

  while (1)
  {
    
  }
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}
void GPIO_Init(void)
{
	// Clock Enable: GPIOA/B/C
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN;

	/* INITIALIZE PINS
	 * PA0: TIM2_CH1_ETR (AF2)
	 * PA1: TIM2_CH2 (AF2)
	 * PA2: TIM2_CH3 (AF2)
	 * PA3: TIM2_CH4 (AF2)
	 * PB5: LED - OUTPUT, PP, LOW
	 * PB6: LED - OUTPUT, PP, LOW
	 * PB7: LED - OUTPUT, PP, LOW
	 * PB13: SPI2_SCK (AF0)
	 * PB14: SPI2_MISO (AF0)
	 * PB15: SPI2_MOSI (AF0)
	 * PC0: CS - OUTPUT,PP,LOW
	 * PC1: INT1 - INPUT
	 * PC2: DRDY/INT2 - INPUT
	 * PC6: TIM3_CH1 (AF0)
	 * PC7: TIM3_CH2 (AF0)
	 * PC8: TIM3_CH3 (AF0)
	 * PC9: TIM3_CH4 (AF0)
	 * PC10: USART4_TX (AF0)
	 * PC11: USART4_RX (AF0)
	 */
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
  GPIOB->MODER |= GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER13_1
									| GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
	GPIOC->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1
									| GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;

	GPIOA->AFR[0] |= (0x2)|(0x2 << GPIO_AFRL_AFRL1_Pos)|(0x2 << GPIO_AFRL_AFRL2_Pos)|(0x2 << GPIO_AFRL_AFRL3_Pos);
  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFRH0 | GPIO_AFRH_AFRH5 | GPIO_AFRH_AFRH6 | GPIO_AFRH_AFRH7);
	GPIOC->AFR[0] &= ~(GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7);
	GPIOC->AFR[1] &= ~(GPIO_AFRH_AFRH0 | GPIO_AFRH_AFRH1 | GPIO_AFRH_AFRH2 | GPIO_AFRH_AFRH3);

}

void GYRO_Init(void)
{
	// L3GD20HTR Initialization
}

void GYRO_Write(uint8_t TxData, uint8_t WriteAddr)
{
  // SPI Transmission Step 1: Bring CS (serial port enable) LOW
  SPI2->CR1 &= ~(SPI_CR1_SPE);
	
	// SPI Transmission Step 2: Format and transmit data
  uint8_t cmd[2] = {0x00 | WriteAddr, TxData};
	uint8_t *pData = cmd;
	uint8_t count = 2;
	
  while(count > 0)
	{
		if(SPI2->SR & SPI_SR_TXE)
		{
			SPI2->DR = *pData;
			pData += sizeof(uint8_t);
			count--;
		}
	}
	// SPI Transmission Step 3: Bring CS (serial port enable) HIGH
  SPI2->CR1 |= SPI_CR1_SPE;
}

void GYRO_Read(uint8_t* pBuffer,uint8_t ReadAddr, uint16_t NumByteToRead)
{
  // SPI Transmission Step 1: Bring CS (serial port enable) LOW
  SPI2->CR1 &= ~(SPI_CR1_SPE);

  // SPI Transmission Step 2: Format and read data

  // SPI Transmission Step 3: Bring CS (serial port enable) HIGH
  SPI2->CR1 |= SPI_CR1_SPE;
}

void SPI_Init(void)
{
	/* SPI Initialization
	 * Default (Reset): CPOL (Low), CPHA (1st Edge), 8bit Data size, (tx/rx) 1st bit = MSB
	 * CR1: Master, Baud Rate: fPCLK/16 (3MHz)
	 * CR2: Enable interrupts (Error, RXNE, TXE)
	 */
	SPI2->CR1 = SPI_CR1_MSTR | (0x7 << SPI_CR1_BR_Pos);
	SPI2->CR2 |= SPI_CR2_ERRIE | SPI_CR2_RXNEIE | SPI_CR2_TXEIE;
	SPI2->CR1 |= SPI_CR1_SPE;
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
