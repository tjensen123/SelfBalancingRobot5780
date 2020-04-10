
#include "main.h"
void SystemClock_Config(void);

volatile int16_t x_value;
volatile int8_t x_l;
volatile int8_t x_h = 0;
volatile int16_t y_value;
volatile int8_t y_l;
volatile int8_t y_h;


void GPIO_Init(void);
void I2C2_Init(void);
void Who_Am_I(void);
void MEMS_Tx(uint8_t NBYTES, uint8_t* TxData);
int* MEMS_Rx(uint8_t NBYTES, int* RxData);

int main(void)
{
  HAL_Init();
	GPIO_Init();
	I2C2_Init();
	SystemClock_Config();
	
	// Who_Am_I();
	
	// Gyroscope: Set PD,XEN,YEN in CTRL1
	unsigned char buf[2];
	buf[0] = 0x20; // Slave Register ADDR (CTRL1)
	buf[1] = 0xB;  // PD,XEN,YEN
	MEMS_Tx(2,buf);
	
  while (1)
  {		
		
		// Check to see if new X,Y DATA
		unsigned char SR = 0x27;
		MEMS_Tx(1,&SR);
		int Status[1];
		MEMS_Rx(1,Status);
		
		// NEW X DATA
		if(*Status & (1 << 0)){
			/* Access X Reg (WRITE)
			 * Note: 7b Reg Addr = 0x28, making MSb = 1 allows for multiple writes!
			 * We need multiple writes because there is X-L & X-H */
			unsigned char x_addr = 0xA8;
			MEMS_Tx(1,&x_addr);
			
			// Read X Reg
			// NOTE: Values given in 2's complement. 16b 2's range: (-32768 , 32767)
			// NOTE: Values broken up into 8b, because that's how much RXDR can hold. Need to COMBINE
			int x_val[2];
			MEMS_Rx(2,x_val);
			x_l = x_val[0];
			x_h = x_val[1];
			x_value = (x_h << 8)|(x_l);
		}
		
		// NEW Y DATA
		if(*Status & (1 << 1)){
			/* Access Y Reg (WRITE)
			 * Note: 7b Reg Addr = 0x2A, making MSb = 1 allows for multiple writes!
			 * We need multiple writes because there is X-L & X-H */
			unsigned char y_addr = 0xAA;
			MEMS_Tx(1,&y_addr);
			
			// Read Y Reg
			// NOTE: Values given in 2's complement. 16b 2's range: (-32768 , 32767)
			// NOTE: Values broken up into 8b, because that's how much RXDR can hold. Need to COMBINE
			int y_val[2];
			MEMS_Rx(2,y_val);
			y_l = y_val[0];
			y_h = y_val[1];
			y_value = (y_h << 8) | y_l;
		}
		
		// LED Indication of PITCH (X) and ROLL (Y)
		// Because of noise, need a threshold (3000)
		
		if(x_value > 3000){
			GPIOC->ODR &= ~(GPIO_ODR_8);
			GPIOC->ODR |= GPIO_ODR_9;
		}
		else if(x_value < -3000){
			GPIOC->ODR &= ~(GPIO_ODR_9);
			GPIOC->ODR |= GPIO_ODR_8;
		}
		if(y_value > 3000){
			GPIOC->ODR &= ~(GPIO_ODR_7);
			GPIOC->ODR |= GPIO_ODR_6;
		}
		else if(y_value < -3000){
			GPIOC->ODR &= ~(GPIO_ODR_6);
			GPIOC->ODR |= GPIO_ODR_7;
		}
		
		HAL_Delay(100);
  }

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

void GPIO_Init(void)
{
	// Clock Enable: GPIOB/C
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN;

	/* INITIALIZE PINS
	 * PB11: I2C SDA,AF1, OD
	 * PB13: I2C SCL,AF5, OD
	 * PB14: OUTPUT,PP,HIGH
	 * PB15: INPUT
	 * PC0: OUTPUT,PP,HIGH
	 * LEDS (PC6,7,8,9): OUTPUT,PP,LOWSPEED	
	 */
	GPIOB->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_0;
	GPIOC->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER6_0| GPIO_MODER_MODER7_0| GPIO_MODER_MODER8_0| GPIO_MODER_MODER9_0;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_13;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_0|GPIO_PUPDR_PUPDR13_0;
	GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFRH3_Pos)|(0x5 << GPIO_AFRH_AFRH5_Pos);
	GPIOB->ODR |= GPIO_ODR_14;
	GPIOC->ODR |= GPIO_ODR_0;
}

void I2C2_Init(void)
{
	// Clock Enable: I2C2
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// I2C2 INITIALIZATION
	// 100kHz config for our 8 MHz processor
	I2C2->TIMINGR = (1 << I2C_TIMINGR_PRESC_Pos)|(0x13 << I2C_TIMINGR_SCLL_Pos)|(0xF << I2C_TIMINGR_SCLH_Pos)|(0x2 << I2C_TIMINGR_SDADEL_Pos)|(0x4 << I2C_TIMINGR_SCLDEL_Pos);
	I2C2->CR1 = I2C_CR1_PE;
}
void Who_Am_I(void)
{
	/* FUNCTION DESCRIPTION:
	All LEDs blink: WHO_AM_I slave register holds the value of D4 (expected)
	Only red LED blinks: WHO_AM_I isn't D4
	Solid Red: NACK from WRITE
	Solid Red/Blue: NACK from READ
	*/
	
	// Request a WRITE to SLAVE @ 0x6B
	I2C2->CR2 &= ~(I2C_CR2_NBYTES | I2C_CR2_SADD);
	I2C2->CR2 |= (1 << 16)|(0x6B << 1)|I2C_CR2_START;
	
	// Poll TXIS & NACKF
	while(!(I2C2->ISR & I2C_ISR_TXIS)){
		// If request not acknowledged...
		if(I2C2->ISR & I2C_ISR_NACKF){
					GPIOC->ODR |= GPIO_ODR_6;	// Red
		}
	}
	
	// Write WHO_AM_I to TXDR
	I2C2->TXDR = 0x0F;

	// Poll TC
	while(!(I2C2->ISR & I2C_ISR_TC));
	
	// Request a READ from SLAVE @ 0x6B
	I2C2->CR2 &= ~(I2C_CR2_NBYTES | I2C_CR2_SADD);
	I2C2->CR2 |= (1 << 16)|(0x6B << 1)|I2C_CR2_RD_WRN|I2C_CR2_START;

	// Poll RXNE & NACKF
	while(!(I2C2->ISR & I2C_ISR_RXNE)){
		// If request not acknowledged...
		if(I2C2->ISR & I2C_ISR_NACKF){
			GPIOC->ODR |= GPIO_ODR_6|GPIO_ODR_7;	// Red + Blue
		}
	}
	
	// Poll TC
	while(!(I2C2->ISR & I2C_ISR_TC));
	
	if(I2C2->RXDR == 0xD4){
		while(1) {
			GPIOC->ODR ^= GPIO_ODR_6|GPIO_ODR_7|GPIO_ODR_8|GPIO_ODR_9;
			HAL_Delay(250); //BLINK ALL
		}
	}
	else{
		while(1) {
			GPIOC->ODR ^= GPIO_ODR_6;
			HAL_Delay(250); //BLINK RED
		}
	}
	
	//Release Bus
	I2C2->CR2 |= I2C_CR2_STOP;
}
void MEMS_Tx(uint8_t NBYTES, uint8_t* TxData){
	
	I2C2->CR2 &= ~(I2C_CR2_NBYTES | I2C_CR2_SADD | I2C_CR2_RD_WRN | I2C_CR2_START);
	I2C2->CR2 |= (NBYTES << 16)|(0x6B << 1) | I2C_CR2_START;
	
	for(int i=0;i<NBYTES;i++){
		// Poll TXIS & NACKF
		while(!(I2C2->ISR & I2C_ISR_TXIS)){
			// If request not acknowledged...
			if(I2C2->ISR & I2C_ISR_NACKF){
					GPIOC->ODR |= GPIO_ODR_6;	// Red
			}
		}
		// ACK: Write next byte to TXDR
		I2C2->TXDR = TxData[i];
	}
	
	// Poll TC
	while(!(I2C2->ISR & I2C_ISR_TC));
}

int* MEMS_Rx(uint8_t NBYTES, int* RxData){
	
	I2C2->CR2 &= ~(I2C_CR2_NBYTES | I2C_CR2_SADD | I2C_CR2_RD_WRN | I2C_CR2_START);
	I2C2->CR2 |= (NBYTES << 16)|(0x6B << 1) | I2C_CR2_RD_WRN | I2C_CR2_START;
	
	for(int i=0;i<NBYTES;i++){
		// Poll TXIS & NACKF
		while(!(I2C2->ISR & I2C_ISR_RXNE)){
			// If request not acknowledged...
			if(I2C2->ISR & I2C_ISR_NACKF){
					GPIOC->ODR |= GPIO_ODR_6;	// Red
			}
		}
		RxData[i] = I2C2->RXDR;
	}
	
	// Poll TC
	while(!(I2C2->ISR & I2C_ISR_TC));
	
	return RxData;
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
