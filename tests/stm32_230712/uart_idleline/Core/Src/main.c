/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdlib.h> // For atof
#include <string.h> // For memset and strtok
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// This is to read the angle from the magnetic encoder AS5048A
uint16_t read_angle(void)
{
  uint8_t txData[2] = {0xFF, 0xFF}; // Transmit buffer (command to request angle)
  uint8_t rxData[2] = {0x00, 0x00}; // Receive buffer
  uint16_t angle = 0; // Angle

  // Make sure the Chip Select pin is high before starting a new transmission
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Wait for previous transmissions to finish
  //while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}; // or HAL_SPI_GetState(&hspi2) based on the SPI peripheral used

  // Pull the Chip Select pin low to start a new transmission
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  // Transmit the command and receive the result
  HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 50); // or HAL_SPI_TransmitReceive(&hspi2) based on the SPI peripheral used

  // Push the Chip Select pin high to end the transmission
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  // Combine the two bytes received into one 16-bit number
  angle = ((rxData[0] << 8) | rxData[1]) & 0x3FFF; // The AS5048A only uses the lower 14 bits of this 16-bit number

  return angle;
}


//This is for the motor driver pca9685
void pca9685_init(I2C_HandleTypeDef *hi2c, uint8_t address)
{
#define PCA9685_MODE1 0x00
uint8_t initStruct[2];
uint8_t prescale = 0x144; // hardcoded
HAL_I2C_Master_Transmit(hi2c, address, PCA9685_MODE1, 1, 1);
uint8_t oldmode = 0; // hardcoded
// HAL_I2C_Master_Receive(hi2c, address, &oldmode, 1, 1);
uint8_t newmode = ((oldmode & 0x7F) | 0x10);
initStruct[0] = PCA9685_MODE1;
initStruct[1] = newmode;
HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
initStruct[1] = prescale;
HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
initStruct[1] = oldmode;
HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
HAL_Delay(5);
initStruct[1] = (oldmode | 0xA1);
HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
}

void pca9685_pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint16_t on, uint16_t off)
{
uint8_t outputBuffer[5] = {0x06 + 4*num, on, (on >> 8), off, (off >> 8)};
HAL_I2C_Master_Transmit(&hi2c1, address, outputBuffer, 5, 1);
}


//This is for the ESP8266 conection
#define RxBuf_SIZE 520
#define Theta_SIZE 100


uint8_t RxBuf[RxBuf_SIZE];

uint16_t oldPos = 0;
uint16_t newPos = 0;
uint8_t interesting[520];

int isOK = 0;
int new_data = 0;
int blink_wait = 1000;
int new_interesting = 0;
float tetha[Theta_SIZE];


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	int i;
	for (i=Size;i<RxBuf_SIZE;i++){
	  RxBuf[i] = 0;
	}
	if (huart->Instance == USART1)
	{
		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

	}

	int j=0;
	memset(interesting, '#', sizeof(interesting)); // Initialize the array with '#' character
	for (int i=0; i<Size; i++)
	{
		if (RxBuf[i] == '$')
		{
			new_interesting = 1;

			while(RxBuf[i+j+1] != '!')
				{
					interesting[j] = RxBuf[i+j+1];
					j++;
				}

		  break;
		}
	}

	int tetha_index = 0;
	if (new_interesting == 1)
	{
		interesting[j] = '\0'; // Null terminate the string

		char* token = strtok(interesting, ",");
		while (token != NULL)
		{
			if(tetha_index < Theta_SIZE)
			{
				tetha[tetha_index++] = atof(token);
				token = strtok(NULL, ",");
			}
			else
			{
				break; // blink_wait array is full
			}
		}
		new_interesting = 0;
		new_data = 1;
		//HAL_Delay(5000);
	}
}


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
  pca9685_init(&hi2c1, 0x80);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuf, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Pa que se desapendeje
  HAL_Delay(2000);
  int up=1;
  int i = 0;
  while (i<600){
	  pca9685_pwm(&hi2c1, 0x80, 3, 0, i);
	  i = i+up;
	  HAL_Delay(1);
  }
  HAL_Delay(5000);
  pca9685_pwm(&hi2c1, 0x80, 3, 0, 0);
  HAL_Delay(5000);
  /*HAL_UART_Transmit(&huart1, "AT+CWJAP=\"batbot\",\"12345679\"\r\n", sizeof("AT+CWJAP=\"batbot\",\"12345679\"\r\n"), 1000);
  HAL_Delay(10000);

  HAL_UART_Transmit(&huart1, "AT+CWMODE=1\r\n", sizeof("AT+CWMODE=1\r\n"), 1000);
  HAL_Delay(2000);

  HAL_UART_Transmit(&huart1, "AT+CIFSR\r\n", sizeof("AT+CIFSR\r\n"), 1000);
  HAL_Delay(2000);

  HAL_UART_Transmit(&huart1, "AT+CIPMUX=1\r\n", sizeof("AT+CIPMUX=1\r\n"), 1000);
  HAL_Delay(2000);

  HAL_UART_Transmit(&huart1, "AT+CIPSERVER=1,80\r\n", sizeof("AT+CIPSERVER=1,80\r\n"), 1000);
  HAL_Delay(2000);
  HAL_Delay(5000);

  HAL_UART_Transmit(&huart1, "AT+CIPSTA=\"192.168.194.81\"\r\n",

  sizeof("AT+CIPSTA=\"192.168.194.81\"\r\n"), 1000);
  HAL_Delay(2000);

  HAL_UART_Transmit(&huart1, "AT+CIFSR\r\n", sizeof("AT+CIFSR\r\n"), 1000);
  HAL_Delay(2000);

  HAL_UART_Transmit(&huart1, "AT+CWJAP=\"batbot\",\"12345679\"\r\n", sizeof("AT+CWJAP=\"batbot\",\"12345679\"\r\n"), 1000);
  HAL_Delay(17000);

  HAL_UART_Transmit(&huart1, "AT+CWMODE=1\r\n", sizeof("AT+CWMODE=1\r\n"), 1000);
  HAL_Delay(2000);*/
 //HAL_Delay(10000);


  HAL_UART_Transmit(&huart1, "AT+CIPMUX=1\r\n", sizeof("AT+CIPMUX=1\r\n"), 1000);
  HAL_Delay(2000);

  HAL_UART_Transmit(&huart1, "AT+CIPSERVER=1,80\r\n", sizeof("AT+CIPSERVER=1,80\r\n"), 1000);
  HAL_Delay(2000);

  HAL_UART_Transmit(&huart1, "AT+CIFSR\r\n", sizeof("AT+CIFSR\r\n"), 1000);
  HAL_Delay(2000);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
  HAL_Delay(1000);

  float max_time = 7000;
  while (1)
  {
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	  // Start counting time
	  HAL_TIM_Base_Stop(&htim2);
	  __HAL_TIM_SET_COUNTER(&htim2, 0);
	  HAL_TIM_Base_Start(&htim2);

	  if(new_data==1){
		   // Theta
		  int motor = tetha[0]; // Theta 0:  150-400
		  int attack_angle = tetha[1]; // Theta 1: 1150-1450 (45°-0°)
		  int neutral_state = tetha[2]; // Theta 2: 1213-1637
		  int amplitude = tetha[3]; // Theta 3: 0-213


		  // Initial values
		  uint16_t angle = 0;
		  uint16_t angle_last = 0;
		  int flap = 0;
		  int legs = 0;
		  int up=1;
		  int i = 0;
		  float cyc;

		 // Attack angle
		  pca9685_pwm(&hi2c1, 0x80, 4, 0, attack_angle);
		  HAL_Delay(1000);
		  // Start the motor slowly (if not it stops)
		  while (i<motor){
			  pca9685_pwm(&hi2c1, 0x80, 3, 0, i);
			  i = i+up;
			  HAL_Delay(1);
		  }

		  // Start counting time
		  HAL_TIM_Base_Stop(&htim2);
		  __HAL_TIM_SET_COUNTER(&htim2, 0);
		  HAL_TIM_Base_Start(&htim2);

		  // Used to calculate derivative which gives stroke direction
		  angle_last = read_angle();

		  // Start test
		  while(__HAL_TIM_GET_COUNTER(&htim2)<= max_time){
			  angle = read_angle(); // real extreme values 6612-9702 down-up
			  cyc = (float)(angle-6600)/(9720-6600);  // down:0 up:1
			  if( cyc>0.5){flap = 1000;} // after half way up star extending
			  else if(cyc<0.2){flap = 1850;} // 20% before reaching down start folding
			  else if(angle_last<angle){flap = 1850;} //up-stroke folding
			  else{flap = 1000;} // down-stroke extend

			  // legs down (neutral_state-amplitude) - up(neutral_state+amplitude): 1000-1850
			  legs = (neutral_state-amplitude)*(1-cyc) + (neutral_state+amplitude)*cyc;
			  pca9685_pwm(&hi2c1, 0x80, 0, 0, legs);
			  pca9685_pwm(&hi2c1, 0x80, 1, 0, legs);
			  pca9685_pwm(&hi2c1, 0x80, 2, 0, flap); // Extended 1000, folded 1850
			  angle_last = angle;

		  }
		  new_data = 0;
		  pca9685_pwm(&hi2c1, 0x80, 0, 0, 1400);
		  pca9685_pwm(&hi2c1, 0x80, 1, 0, 1400);
		  pca9685_pwm(&hi2c1, 0x80, 2, 0, 1400);
		  pca9685_pwm(&hi2c1, 0x80, 3, 0, 0);
		  pca9685_pwm(&hi2c1, 0x80, 4, 0, 1450);
		  HAL_Delay(1000);
		  //pca9685_pwm(&hi2c1, 0x80, 4, 0, 0); // neutral state of motors
	  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
