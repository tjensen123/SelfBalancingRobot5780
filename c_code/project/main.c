#include "main.h"

SPI_HandleTypeDef hspi2;

void SystemClock_Config(void);
void GPIO_Init(void);
void SPI2_Init(void);
void USART4_Init(void);
void TIMER_Init(void);
void GYRO_Init(void);
float GYRO_Read_X(float sensitivity);
float GYRO_Read_Y(float sensitivity);
void GYRO_Write(uint8_t TxData, uint8_t WriteAddr);
void GYRO_Read(uint8_t* RxData,uint8_t ReadAddr, uint16_t Bytes);
void R_Forward(uint16_t speed);
void R_Reverse(uint16_t speed);
void L_Forward(uint16_t speed);
void L_Reverse(uint16_t speed);
void R_Stop(void);
void L_Stop(void);

int left=4;
int right=4;
int storeLeft=4;
int storeRight=4;
volatile char data[5]= "L4R4:";
volatile char dataChar;
volatile int i = 0;
volatile int16_t xdata = 0;
volatile int16_t ydata = 0;
float sensitivity_245 = 114.285f;

int main(void)
{

  HAL_Init();
  SystemClock_Config();
	GPIO_Init();
  SPI2_Init();
	USART4_Init();
	GYRO_Init();
	TIMER_Init();
	
	uint8_t thresh = 10;
	
	// Get the address of the NEW DATA register in the GYRO
	uint8_t status[1];
	GYRO_Read(status,0x27,1);
	
//	for(int j=4000;j>0;j--)
//	{
//		R_Forward(j);
//		if(j==400)while(1);
//		HAL_Delay(1);
//	}
	
  while (1)
  {
		// Only need 1 axis depending how board is oriented if used for self-balancing
		// This iteration uses the GYRO as another controller for movement
		
		// New X-Data
		if(*status & (1<<0)){
			xdata = (int16_t)GYRO_Read_X(sensitivity_245);
		}
		// New Y-Data
		if(*status & (1<<1)){
			ydata = (int16_t)GYRO_Read_Y(sensitivity_245);
		}
		// Falling Forward
		if(xdata > thresh){
			GPIOC->ODR &= ~(GPIO_ODR_8);
			GPIOC->ODR |= GPIO_ODR_9;
			R_Forward(xdata);
			L_Forward(xdata);
		}
		// Falling Backward
		else if(xdata < -thresh){
			GPIOC->ODR &= ~(GPIO_ODR_9);
			GPIOC->ODR |= GPIO_ODR_8;
			R_Reverse(xdata*-1);
			L_Reverse(xdata*-1);
		}
		// Rotate Right
		if(ydata > thresh){
			GPIOC->ODR &= ~(GPIO_ODR_7);
			GPIOC->ODR |= GPIO_ODR_6;
			R_Reverse(ydata);
			L_Forward(ydata);
		}
		// Rotate Left
		else if(ydata < -thresh){
			GPIOC->ODR &= ~(GPIO_ODR_6);
			GPIOC->ODR |= GPIO_ODR_7;
			R_Forward(ydata*-1);
			L_Reverse(ydata*-1);
		}
		else GPIOC->ODR = 0;
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
void GPIO_Init(void)
{
	// Clock Enable: GPIO A/B/C
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN;

//	// SPI2 - GYRO
//	// SCK - PB13, MISO - PB14, MOSI - PB15 (AF0)
//	// CS - PC0, INT1 - PC1, INT2 - PC2
//  GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
//	GPIOB->OSPEEDR |= 0x3F << 26;
	GPIOC->MODER |= GPIO_MODER_MODER0_0| GPIO_MODER_MODER1_1;

	// USART4 - BLUETOOTH
	// PC10,11 (AF0)
	GPIOC->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	
	// LEDs
	// PC6/7/8/9 : Output, PP 
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
	
	// TIM1
	// CH 1: PA8 (AF2)
	// CH 2: PA9(AF2)
	GPIOA->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
	GPIOA->AFR[1] |= (0x2)|(0x2 << GPIO_AFRL_AFRL1_Pos);	
	
	// TIM2
	// CH 1: PA0 (AF2)
	// CH 2: PA1 (AF2)
	// CH 3: PB10 (AF2)
	// CH 4: PB11 (AF2)
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
	GPIOA->AFR[0] |= (0x2)|(0x2 << GPIO_AFRL_AFRL1_Pos);
	GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	GPIOB->AFR[1] |= (0x2 << GPIO_AFRH_AFRH2_Pos)|(0x2 << GPIO_AFRH_AFRH3_Pos);
	
	// TIM3
	// CH 1: PB4 (AF1)
	// CH 2: PB5 (AF1)
	GPIOB->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
	GPIOB->AFR[0] |= (0x1 << GPIO_AFRL_AFRL4_Pos)|(0x1 << GPIO_AFRL_AFRL5_Pos);
}

void SPI2_Init(void)
{
	/* SPI2 Initialization
	 * Enable Clock
	 * Default (Reset): CPOL (Low), CPHA (1st Edge), 8bit Data size, (tx/rx) 1st bit = MSB
	 * CR1: Master, Baud Rate: fPCLK/8 (3MHz)
	 * CR2: Enable interrupts (Error, RXNE, TXE)
	
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	
	SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_SSM | SPI_CR1_SSI;
	SPI2->CR2 |= SPI_CR2_ERRIE | SPI_CR2_RXNEIE | SPI_CR2_TXEIE;
	SPI2->CR1 |= SPI_CR1_SPE;
	*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}
void USART4_Init(void)
{
	/* USART4 Initialization
	 * Enable Clock
	 * Default (Reset): 
	 */
	RCC->APB1ENR |= RCC_APB1ENR_USART4EN;
	USART4->BRR = HAL_RCC_GetHCLKFreq()/9600;
	USART4->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE;
	USART4->CR1 |= USART_CR1_UE;
	
	NVIC_EnableIRQ(USART3_4_IRQn);	// Enable interrupt
	NVIC_SetPriority(USART3_4_IRQn,1);	// Set priority
}

void GYRO_Init(void)
{
	// L3GD20HTR Initialization
	GYRO_Write(0x08,0x22); // CTRL3: INT2/DRDY Interrupt enable
	GYRO_Write(0x80,0x23); // CTRL4: 250 dps, Block Data Update (until MSB & LSB reading)
	GYRO_Write(0x0F,0x20); // CTRL1: Data rate = 100Hz, BW = 12.5 Hz, XYZ enable, Power ON
}
void TIMER_Init(void)
{
	// Full-Step bipolar stepper motor driver
	// Speed controlled by FREQUENCY which is set by PSC
	// Timer 2 controls RIGHT MOTOR
	//		+ CH 1/3 (coil 1), CH 2/4 (coil 2)
	//    + CH1 = ~CH3, CH2 = ~CH4
	// Timer 1/3 controls LEFT MOTOR
	//		+ Timer 1
	//				- CH1/2 (coil 1)
	//				- CH1 = ~CH2
	//		+ Timer 3
	//				- CH1/2 (coil 2)
	//				- CH1 = ~CH2
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	
	// Timer1
	TIM1->CR1 = 0;
	TIM1->CCMR1 = 0;
	TIM1->CCMR2 = 0;
	TIM1->CCER = 0;
	TIM1->PSC = 65500;
	TIM1->ARR = 300;
	TIM1->CCMR1 |= 0x3 << TIM_CCMR1_OC2M_Pos | TIM_CCMR1_OC2PE |0x3 << TIM_CCMR1_OC1M_Pos | TIM_CCMR1_OC1PE; // CH1/2 - TOGGLE MODE
	TIM1->CCER	|= TIM_CCER_CC2P;
	TIM1->CCER	|= TIM_CCER_CC1E | TIM_CCER_CC2E; // Output enabled for CH1/!CH2
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	
	// Timer2
	TIM2->CR1 = 0;
	TIM2->CCMR1 = 0;
	TIM2->CCMR2 = 0;
	TIM2->CCER = 0;
	TIM2->PSC = 1100;
	TIM2->ARR = 300;
	TIM2->CCMR1 |= 0x3 << TIM_CCMR1_OC1M_Pos | 0x3 << TIM_CCMR1_OC2M_Pos; // CH1&2 - TOGGLE MODE
	TIM2->CCMR2 |= 0x3 << TIM_CCMR2_OC3M_Pos | 0x3 << TIM_CCMR2_OC4M_Pos; // CH3&4 - TOGGLE MODE
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
	TIM2->CCER	|= TIM_CCER_CC4P | TIM_CCER_CC2P;
	TIM2->CCER	|= TIM_CCER_CC1E | TIM_CCER_CC2E| TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;
	
	// Timer3
	TIM3->CR1 = 0;
	TIM3->CCMR1 = 0;
	TIM3->CCER = 0;
	TIM3->PSC = 65500;
	TIM3->ARR = 300;
	TIM3->CCMR1 |= 0x3 << TIM_CCMR1_OC1M_Pos | 0x3 << TIM_CCMR1_OC2M_Pos | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
	TIM3->CCER	|= TIM_CCER_CC2P;
	TIM3->CCER	|= TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;

}
void R_Forward(uint16_t speed)
{
	// SEQUENCE: CH1/2, CH3/4
	TIM2->CR1	&= ~TIM_CR1_CEN;

	TIM2->PSC = (speed)*675;
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
	TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk | TIM_CCMR2_OC4M_Msk);
	TIM2->CCMR1 |= 0x3 << TIM_CCMR1_OC1M_Pos | 0x3 << TIM_CCMR1_OC2M_Pos; // CH1&2 - TOGGLE MODE
	TIM2->CCMR2 |= 0x3 << TIM_CCMR2_OC3M_Pos | 0x3 << TIM_CCMR2_OC4M_Pos; // CH3&4 - TOGGLE MODE
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 150;
	TIM2->CCR4 = 150;

	TIM2->CR1	|= TIM_CR1_CEN;
}
void R_Reverse(uint16_t speed)
{
	// SEQUENCE: CH3/4, CH1/2
	TIM2->CR1	&= ~TIM_CR1_CEN;

	TIM2->PSC = (speed)*675;
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
	TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk | TIM_CCMR2_OC4M_Msk);
	TIM2->CCMR1 |= 0x3 << TIM_CCMR1_OC1M_Pos | 0x3 << TIM_CCMR1_OC2M_Pos; // CH1&2 - TOGGLE MODE
	TIM2->CCMR2 |= 0x3 << TIM_CCMR2_OC3M_Pos | 0x3 << TIM_CCMR2_OC4M_Pos; // CH3&4 - TOGGLE MODE
	TIM2->CCR1 = 150;
	TIM2->CCR2 = 150;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;	

	TIM2->CR1	|= TIM_CR1_CEN;
}
void L_Forward(uint16_t speed)
{
	// SEQUENCE: CH1/2(TIM3), CH1/2(TIM1)
	TIM1->CR1	&= ~TIM_CR1_CEN;
	TIM3->CR1	&= ~TIM_CR1_CEN;

	TIM1->PSC = (speed)*675;
	TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
	TIM1->CCMR1 |= 0x3 << TIM_CCMR1_OC1M_Pos | 0x3 << TIM_CCMR1_OC2M_Pos; // CH1&2 - TOGGLE MODE
	TIM1->CCR1 = 150;
	TIM1->CCR2 = 150;
	TIM3->PSC = (speed)*675;
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
	TIM3->CCMR1 |= 0x3 << TIM_CCMR1_OC1M_Pos | 0x3 << TIM_CCMR1_OC2M_Pos; // CH1&2 - TOGGLE MODE
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;

	TIM3->CR1	|= TIM_CR1_CEN;
	TIM1->CR1	|= TIM_CR1_CEN;
}
void L_Reverse(uint16_t speed)
{
	// SEQUENCE: CH1/2(TIM1), CH1/2(TIM3)
	TIM1->CR1	&= ~TIM_CR1_CEN;
	TIM3->CR1	&= ~TIM_CR1_CEN;

	TIM1->PSC = (speed)*675;
	TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
	TIM1->CCMR1 |= 0x3 << TIM_CCMR1_OC1M_Pos | 0x3 << TIM_CCMR1_OC2M_Pos; // CH1&2 - TOGGLE MODE
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM3->PSC = (speed)*675;
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
	TIM3->CCMR1 |= 0x3 << TIM_CCMR1_OC1M_Pos | 0x3 << TIM_CCMR1_OC2M_Pos; // CH1&2 - TOGGLE MODE
	TIM3->CCR1 = 150;
	TIM3->CCR2 = 150;

	TIM3->CR1	|= TIM_CR1_CEN;
	TIM1->CR1	|= TIM_CR1_CEN;
}
void R_Stop(void)
{
	// This function stops all movement by setting each channel LOW for the entire counter sequence	
	TIM2->CR1	&= ~TIM_CR1_CEN;
	
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
	TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M_Msk | TIM_CCMR2_OC4M_Msk);
	TIM2->CCMR1 |= 0x7 << TIM_CCMR1_OC1M_Pos | 0x7 << TIM_CCMR1_OC2M_Pos;
	TIM2->CCMR2 |= 0x7 << TIM_CCMR2_OC3M_Pos | 0x7 << TIM_CCMR2_OC4M_Pos;
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;

	TIM2->CR1	|= TIM_CR1_CEN;
}
void L_Stop(void)
{
	TIM1->CR1	&= ~TIM_CR1_CEN;
	TIM3->CR1	&= ~TIM_CR1_CEN;

	TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
	TIM1->CCMR1 |= 0x7 << TIM_CCMR1_OC1M_Pos | 0x7 << TIM_CCMR1_OC2M_Pos; // CH1&2 - TOGGLE MODE
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
	TIM3->CCMR1 |= 0x7 << TIM_CCMR1_OC1M_Pos | 0x7 << TIM_CCMR1_OC2M_Pos; // CH1&2 - TOGGLE MODE
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;

	TIM3->CR1	|= TIM_CR1_CEN;
	TIM1->CR1	|= TIM_CR1_CEN;
}
float GYRO_Read_X(float sensitivity)
{
	float theta = 0;
	uint8_t x_buff[3]={0,0,0};
	int16_t xdata = 0;
	
	GYRO_Read(x_buff,0x28,2);
	xdata = (int16_t)( (uint16_t)(x_buff[2] << 8) + x_buff[1]);
	
	theta=(float)xdata/sensitivity;
	
	return theta;
}

float GYRO_Read_Y(float sensitivity)
{
	float theta = 0;
	uint8_t y_buff[3]={0,0,0};
	int16_t ydata = 0;
	
	GYRO_Read(y_buff,0x2A,2);
	ydata = (int16_t)( (uint16_t)(y_buff[2] << 8) + y_buff[1]);
	
	theta=(float)ydata/sensitivity;
	
	return theta;
}

void GYRO_Write(uint8_t TxData, uint8_t WriteAddr)
{
  // SPI Transmission Step 1: Bring CS (serial port enable) LOW
  GPIOC->ODR &= ~(GPIO_ODR_0);
	
	// SPI Transmission Step 2: Format and transmit data
  uint8_t cmd[2]= {(0x00 | WriteAddr),TxData};
  HAL_SPI_Transmit(&hspi2,cmd,2,(uint32_t)0x1000);

	// SPI Transmission Step 3: Bring CS (serial port enable) HIGH
  GPIOC->ODR |= GPIO_ODR_0;
}

void GYRO_Read(uint8_t* RxData,uint8_t ReadAddr, uint16_t Bytes)
{
  // SPI Transmission Step 1: Bring CS (serial port enable) LOW
  GPIOC->ODR &= ~(GPIO_ODR_0);

  // SPI Transmission Step 2: Format and read data
	uint8_t cmd[1+Bytes];
	cmd[0]= 0xC0 | ReadAddr;
	
	for(int i = 1; i <= Bytes; i++)
	  cmd[i] = 0x00;
	
	HAL_SPI_TransmitReceive(&hspi2,cmd,RxData,Bytes+1,(uint32_t)0x1000);
	
  // SPI Transmission Step 3: Bring CS (serial port enable) HIGH
  GPIOC->ODR |= GPIO_ODR_0;
	
}

void USART3_4_IRQHandler(void){
	if(USART4->ISR & USART_ISR_RXNE){	// If RX interrupt...
		dataChar = (uint8_t)(USART4->RDR); // Receive data, clear flag
		data[i] = dataChar; //store char
		if(dataChar==':'){
			i=0;
			left = data[1]-48;
			right = data[3]-48;
			
				if(left<4){
					L_Reverse((left+1)*2);
					GPIOC->ODR &= ~(GPIO_ODR_6 | GPIO_ODR_7);
					GPIOC->ODR |= GPIO_ODR_7;
				}
				if(left>4){
					L_Forward((9-left)*2);
					GPIOC->ODR &= ~(GPIO_ODR_6 | GPIO_ODR_7);
					GPIOC->ODR |= GPIO_ODR_6;
				}
				if(left==4){
					L_Stop();
					GPIOC->ODR |= GPIO_ODR_6 | GPIO_ODR_7;
				}
				if(right<4){
					R_Reverse((right+1)*2);
					GPIOC->ODR &= ~(GPIO_ODR_8 | GPIO_ODR_9);
					GPIOC->ODR |= GPIO_ODR_9;
				}
				if(right>4){
					R_Forward((9-right)*2);
					GPIOC->ODR &= ~(GPIO_ODR_8 | GPIO_ODR_9);
					GPIOC->ODR |= GPIO_ODR_8;
				}
				if(right==4){
					R_Stop();
					GPIOC->ODR |= GPIO_ODR_8 | GPIO_ODR_9;
				}
		}
		else{
			i++;
		}
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
