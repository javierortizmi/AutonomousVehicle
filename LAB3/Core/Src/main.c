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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
unsigned short mode = 1;
uint8_t received[7];
uint8_t lineBreak[7];
uint8_t answer[7] = {0,0,0,0,0,0,0};
uint8_t choose[7] = {67, 104, 111, 111, 115, 101, 32};
uint8_t aMode[7] = {97, 32, 109, 111, 100, 101, 32};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void goForward(void);
void goBackward(void);
void Stop(void);
void goRight(void);
void goLeft(void);

void lineBreakFunction(void);
void chooseAMode(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lineBreakFunction(void) {
  lineBreak[0] = 10;
  HAL_UART_Transmit(&huart1, lineBreak, 1, 10000);
  lineBreak[0] = 13;
  HAL_UART_Transmit(&huart1, lineBreak, 1, 10000);
}
void chooseAMode(void) {
  HAL_UART_Transmit(&huart1, choose, 7, 10000);
  HAL_UART_Transmit(&huart1, aMode, 7, 10000);
}

void goForward(void) {
  GPIOA->BSRR = (1 << 11)<<16;
  GPIOA->BSRR = (1 << 12)<<16;
  GPIOB->BSRR = (1 << 8);
  GPIOB->BSRR = (1 << 9);
}

void goBackward(void) {
  GPIOA->BSRR = (1 << 11);
  GPIOA->BSRR = (1 << 12);
  GPIOB->BSRR = (1 << 8)<<16;
  GPIOB->BSRR = (1 << 9)<<16;
}

void Stop(void) {
  GPIOA->BSRR = (1 << 11)<<16;
  GPIOA->BSRR = (1 << 12)<<16;
  GPIOB->BSRR = (1 << 8)<<16;
  GPIOB->BSRR = (1 << 9)<<16;
}

void goRight(void) {
  GPIOA->BSRR = (1 << 11);
  GPIOA->BSRR = (1 << 12)<<16;
  GPIOB->BSRR = (1 << 8)<<16;
  GPIOB->BSRR = (1 << 9);
}

void goLeft(void) {
  GPIOA->BSRR = (1 << 11)<<16;
  GPIOA->BSRR = (1 << 12);
  GPIOB->BSRR = (1 << 8);
  GPIOB->BSRR = (1 << 9)<<16;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // MOTOR 1 CONFIG
  GPIOB->MODER &= ~(1 << (8*2 +1));
  GPIOB->MODER |= (1 << (8*2));
  GPIOA->MODER &= ~(1 << (11*2 +1));  // PA11 as Digital Output(01)
  GPIOA->MODER |= (1 << (11*2));

  // MOTOR 2 CONFIG
  GPIOB->MODER &= ~(1 << (9*2 +1));
  GPIOB->MODER |= (1 << (9*2));
  GPIOA->MODER &= ~(1 << (12*2 +1));  // PA12 as Digital Output(01)
  GPIOA->MODER |= (1 << (12*2));

  HAL_UART_Receive_IT(&huart1, received, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(mode == 1) {
      Stop();
    }
    else if(mode == 2) {
      goForward();
    }
    else if(mode == 3) {
      goBackward();
    }
    else if(mode == 4) {
      goRight();
    }
    else if(mode == 5) {
      goLeft();
    }

    espera(100000);
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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA11 PA12 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(huart, received, 1); // Vuelve a activar Rx por haber acabado el buffer


    if(received[0] == '1') {   // A 1 is received -> STOP
      mode = 1;
      uint8_t answer[7] = {83, 116, 111, 112, 112, 101, 100};
      lineBreakFunction();
      HAL_UART_Transmit(&huart1, answer, 7, 10000);
      lineBreakFunction();
      chooseAMode();
    }
    if(received[0] == '2') {   // A 2 is received -> FORWARD
      mode = 2;
      uint8_t answer[7] = {70, 111, 114, 119, 97, 114, 100};
      lineBreakFunction();
      HAL_UART_Transmit(&huart1, answer, 7, 10000);
      lineBreakFunction();
      chooseAMode();
    }
    if(received[0] == '3') {   // A 3 is received -> BACK
      mode = 3;
      uint8_t answer[7] = {66, 97, 99, 107, 0, 0, 0};
      lineBreakFunction();
      HAL_UART_Transmit(&huart1, answer, 7, 10000);
      lineBreakFunction();
      chooseAMode();
    }
    if(received[0] == '4') {   // A 4 is received -> RIGHT
      mode = 4;
      uint8_t answer[7] = {82, 105, 103, 104, 116, 0, 0};
      lineBreakFunction();
      HAL_UART_Transmit(&huart1, answer, 7, 10000);
      lineBreakFunction();
      chooseAMode();
    }
    if(received[0] == '5') {   // A 5 is received -> LEFT
      mode = 5;
      uint8_t answer[7] = {76, 101, 102, 116, 0, 0, 0};
      lineBreakFunction();
      HAL_UART_Transmit(&huart1, answer, 7, 10000);
      lineBreakFunction();
      chooseAMode();
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
