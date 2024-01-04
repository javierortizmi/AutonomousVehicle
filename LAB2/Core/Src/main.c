/* IN THIS LAB WE WILL LEARN HOW TO USE THE ULTRASONIC SENSOR WITH TIMERS */
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
#include "Utiles_SDM.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

unsigned char cadena[7];
unsigned short distance = 0;                // Measured distance in mm by the ultrasonic sensor
unsigned short triggerTime = 20;            // TIM2->CCR2
unsigned short detectionPeriod = 65000;     // TIM2->CCR3
unsigned short buzzerPeriod = 500;          // TIM2->CCR3
unsigned short startTime = 0;               // Stores the time at which the echo is triggered
int time = 0;
unsigned short buzzerToggleChange = 0;
unsigned short callDistanceFunction = 0, callTriggerFunction = 0, cicloFlag = 0, initialValue = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void calculateDistance(void);   // Declarations of functions
void ultrasonicTrigger(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// TOGGLE BUZZER FUNCTION
void TIM3_IRQHandler(void) {
  if((TIM3->SR & 0x0002) != 0) {
    TIM3->CCR1 += buzzerPeriod;
    buzzerToggleChange++;
    if(buzzerToggleChange>1) buzzerToggleChange=0;
  }
  TIM3->SR &= ~(0x0002);
}

// CALCULATE DISTANCE FUNCTION
void calculateDistance(void) {
  // Apply the formula to calculate the distance (in mm) and store it in a variable
  distance = (0.034 * time) / 2;

}

// TRIGGER FUNCTION
void ultrasonicTrigger(void) {
  TIM3->CNT = 0;                      // Initialize the counter at 0
  GPIOD->BSRR = (1 << 2);             // Set Trigger PIN (1)
}

// HANDLES THE BUZZER AND ECHO TIM2
void TIM2_IRQHandler(void) {
  // Corresponds to the echo flag
  if((TIM2->SR & 0x0004) != 0) {      // CHANNEL 2
    TIM2->CCR2 += detectionPeriod;
    cicloFlag = 1;
    TIM2->SR &= ~(0x0004);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // CONFIG BUZZER GPIO
  GPIOA->MODER &= ~(1 << (1*2 +1));    // PA1 as Digital Output (01)
  GPIOA->MODER |= (1 << (1*2));

  // CONFIG TRIGGER GPIO
  GPIOD->MODER &= ~(1 << (2*2 +1));   // PD2 as Digital Output (01)
  GPIOD->MODER |= (1 << (2*2));

  // CONFIG ECHO GPIO (TIM2 CH1)
  GPIOA->MODER |= (1 << (5*2 +1));    // PA5 as AF (10)
  GPIOA->MODER &= ~(1 << (5*2));
  GPIOA->AFR[0] &= ~(0x00F00000);
  GPIOA->AFR[0] |= (1 << 5*4);        // Select the AF1 for PA5

  // TIM2 CONFIG (TRIGGER ECHO & PPAL LOOP)
  // Select the internal clock
  TIM2->CR1 = 0x0000;     // ARPE = 0 (only for PWM); CEN = 0 (counter disabled for configuration)
  TIM2->CR2 = 0x0000;     // All zeros
  TIM2->SMCR = 0x0000;    // All zeros

  // Counter behavior setting
  TIM2->PSC = 31;            // freq_Counter = 32 MHz / 32 = 1 MHz -->> T_Counter = 1 us
  TIM2->CNT = 0;              // Initialize the counter at 0
  TIM2->ARR = 0xFFFF;         // Maximum value
  TIM2->CCR1 = 0xFFFF;
  TIM2->CCR2 = detectionPeriod;   // CCR3 = 65000 (PPal Loop)

  // Setting IRQ or not
  TIM2->DIER &= ~(0xFF);
  //TIM2->DIER |= (1<<1);       // Enable IRQ for Channel 1 (CC1E)
  TIM2->DIER |= (1<<2);

  // Output mode
  TIM2->CCMR1 &= ~(0xFFFF);   // Clear CCMR1 register
  TIM2->CCMR1 |= 0x0001;      // CC1S = 01 (TIC); CC2S = 00 (TOC); OC2PE = 0 (only for PWM); OC2M = 011 (Toogle)
  TIM2->CCER = 0x000b;

  // Counter enabling
  TIM2->CR1 |= 0x0001;        // CEN = 1 -->> Start counter
  TIM2->EGR |= 0x0001;        // UG = 1 -->> Update all registers
  TIM2->SR = 0x0000;          // Clear counter flags

  // Enabling TIM2_IRQ at NVIC
  NVIC->ISER[0] |= (1 << 28);

  //TIM3 CONFIG (BUZZER)
  // Select the internal clock
  TIM3->CR1 = 0x0000;         // ARPE = 0 (only for PWM); CEN = 0 (counter disabled for configuration)
  TIM3->CR2 = 0x0000;         // All zeros
  TIM3->SMCR = 0x0000;        // All zeros

  //Counter behavior setting
  TIM3->PSC = 31999;             // freq_Counter = 32 MHz / 32000 = 1 kHz -->> T_Counter = 1 ms -->> CCR = 500
  TIM3->CNT = 0;              // Initialize the counter at 0
  TIM3->ARR = 0xFFFF;         // Set to the maximum
  TIM3->CCR1 = buzzerPeriod;   // The first value = 11 = 11 us. Stores the value of the delay. Will be updated

  // Setting IRQ or not
  TIM3->DIER |= (0x0002);    // We disable an IRQ in channel 1

  // Output mode
  TIM3->CCMR1 &= ~(0xFFFF);   // Clear CCMR1 register
  TIM3->CCMR1 = 0x0000;      // CC1S = 0 (TOC); OC1M = 000 (no output); OC1PE = 0 (No preload)
  TIM3->CCER = 0x0000;        // CC1NP = 0; CC1P = 0; CC1E = 0 (Disable output)

  // Counter enabling
  TIM3->CR1 |= 0x0001;        // CEN = 1 -->> Start counter
  TIM3->EGR |= 0x0001;        // UG = 1 -->> Update all registers
  TIM3->SR = 0x0000;          // Clear counter flags

  // Enabling TIM3_IRQ at NVIC
  NVIC->ISER[0] |= (1 << 29);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(cicloFlag == 1) {

      GPIOD->BSRR = (1 << 2);
      initialValue = TIM2->CNT;
      while(TIM2->CNT < (initialValue + triggerTime));
      GPIOD->BSRR = (1 << 2)<<16;


      while((TIM2->SR & 0x0002) == 0);
      TIM2->SR &= ~(0x0002);
      startTime = TIM2->CCR1;

      while((TIM2->SR & 0x0002) == 0);
      TIM2->SR &= ~(0x0002);

      time = TIM2->CCR1 - startTime;
      if(time < 0) time += 0xFFFF;

      calculateDistance();

      cicloFlag = 0;
    }

    if(distance < 10) {
      GPIOA->BSRR = (1 << 1);
    }
    else if(distance >= 10 && distance <= 20) {
      if(buzzerToggleChange == 1) {
        GPIOA->BSRR = (1 << 1);
      }
      if(buzzerToggleChange == 0) {
        GPIOA->BSRR = (1 << 1)<<16;
      }
    }
    else {
      GPIOA->BSRR = (1 << 1)<<16;
    }
    Bin2Ascii(buzzerToggleChange, cadena);
    printf("%s\n\r", cadena);   // Show if the results are what we expected
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 0;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
