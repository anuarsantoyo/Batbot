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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  pca9685_init(&hi2c1, 0x80);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int i=0;
  int up=10;
  //HAL_Delay(5000);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 /*i=1000;
	  pca9685_pwm(&hi2c1, 0x80, 0, 0, 1850); // legs down-up: 1200-1850
	  pca9685_pwm(&hi2c1, 0x80, 1, 0, 1850);
	  pca9685_pwm(&hi2c1, 0x80, 2, 0, 1000); //folding down-up: 1850-1000
	  pca9685_pwm(&hi2c1, 0x80, 3, 0, 0);
	  HAL_Delay(1000);

	  i=1850;
	  pca9685_pwm(&hi2c1, 0x80, 0, 0, 1200);
	  pca9685_pwm(&hi2c1, 0x80, 1, 0, 1200);
	  pca9685_pwm(&hi2c1, 0x80, 2, 0, 1850);
	  pca9685_pwm(&hi2c1, 0x80, 3, 0, i);
	  HAL_Delay(1000);


	  i = i+up;



	  if(i<150){ //from 105 we start to see movement
		  i = i+up;
	  	  }

	  	  if(i<1200||i>1850){
		  up = up*-1;
	  }
	  i = i+up;

	  */
	  /*int count = 0;
	  while(count<5){
		  pca9685_pwm(&hi2c1, 0x80, 0, 0, i);
		  pca9685_pwm(&hi2c1, 0x80, 1, 0, i);
		  pca9685_pwm(&hi2c1, 0x80, 2, 0, i);
		  HAL_Delay(1);
		  if(i<1000||i>1850){
		  up = up*-1;
		  count = count + 1;
		  }
		   i = i+up;
	  }
	  pca9685_pwm(&hi2c1, 0x80, 0, 0, 0);
	  pca9685_pwm(&hi2c1, 0x80, 1, 0, 0);
	  pca9685_pwm(&hi2c1, 0x80, 2, 0, 0);
	  HAL_Delay(1000)

	  i = 0;
	  pca9685_pwm(&hi2c1, 0x80, 0, 0, 0);
	  pca9685_pwm(&hi2c1, 0x80, 1, 0, 0);
	  pca9685_pwm(&hi2c1, 0x80, 2, 0, 0);
	  while (i<600){
		  pca9685_pwm(&hi2c1, 0x80, 3, 0, i);
		  i = i+up;
		  HAL_Delay(1);
	  }

	   pca9685_pwm(&hi2c1, 0x80, 4, 0, i);
	  HAL_Delay(1);
	  if(i<1150||i>1450){
	  up = up*-1;
	  }
	  i = i+up;
	  HAL_Delay(5000);
	  pca9685_pwm(&hi2c1, 0x80, 3, 0, 0);
	  HAL_Delay(5000);
	  while (i<1800){
		  pca9685_pwm(&hi2c1, 0x80, 3, 0, i);
		  i = i+up;
		  HAL_Delay(1);
	  }
	  HAL_Delay(5000);
	  while (i>0){
		  pca9685_pwm(&hi2c1, 0x80, 3, 0, i);
		  i = i-up;
		  HAL_Delay(1);
	  }
	  pca9685_pwm(&hi2c1, 0x80, 4, 0, 1150);
	  HAL_Delay(1000);
	  pca9685_pwm(&hi2c1, 0x80, 4, 0, 0);
	  HAL_Delay(1000);

	  while (i<800){
		  pca9685_pwm(&hi2c1, 0x80, 3, 0, i);
		  i = i+up;
		  HAL_Delay(1);
	  }
	  HAL_Delay(1000);
	  while (i>0){
		  pca9685_pwm(&hi2c1, 0x80, 3, 0, i);
		  i = i-up;
		  HAL_Delay(1);
	  }
	  pca9685_pwm(&hi2c1, 0x80, 4, 0, 1450);
	  HAL_Delay(1000);*/

	  HAL_Delay(500);
	  while (i<1800){
		  pca9685_pwm(&hi2c1, 0x80, 3, 0, i);
		  i = i+up;
		  HAL_Delay(1);
	  }
	  HAL_Delay(500);
	  while (i>0){
		  pca9685_pwm(&hi2c1, 0x80, 3, 0, i);
		  i = i-up;
		  HAL_Delay(1);
	  }

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
