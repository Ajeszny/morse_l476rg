/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
#define TIME_UNIT 1000
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void send(UART_HandleTypeDef *huart, char c) {
	huart->Instance->TDR = c;
}

char get(UART_HandleTypeDef *huart) {
	char data = '\0';
	HAL_UART_Receive(huart, &data, 1, 100);
	return data;
}

char convert_char(char* message, int len) {
	switch (len) {
	case 1:
		return (message[0] == '-')?'t':'e';
	case 2:
		if (message[0] == '-'&&message[1] == '-') {
			return 'm';
		}
		if (message[0] == '-'&&message[1] == '.') {
			return 'n';
		}
		if (message[0] == '.'&&message[1] == '-') {
			return 'a';
		}
		if (message[0] == '.'&&message[1] == '.') {
			return 'i';
		}
	case 3:
		if (message[0] == '-'&&message[1] == '.'&&message[2] == '.') {
			return 'd';
		}
		if (message[0] == '-'&&message[1] == '-'&&message[2] == '.') {
			return 'g';
		}
		if (message[0] == '-'&&message[1] == '.'&&message[2] == '-') {
			return 'k';
		}
		if (message[0] == '-'&&message[1] == '-'&&message[2] == '-') {
			return 'o';
		}
		if (message[0] == '.'&&message[1] == '-'&&message[2] == '.') {
			return 'r';
		}
		if (message[0] == '.'&&message[1] == '.'&&message[2] == '.') {
			return 's';
		}
		if (message[0] == '.'&&message[1] == '.'&&message[2] == '-') {
			return 'u';
		}
		if (message[0] == '.'&&message[1] == '-'&&message[2] == '-') {
			return 'w';
		}
	case 4:
		if (message[0] == '.'&&message[1] == '.'&&message[2] == '-'&&message[3] == '-') {
			return ' ';
		}
		if (message[0] == '-'&&message[1] == '.'&&message[2] == '.'&&message[3] == '.') {
			return 'b';
		}
		if (message[0] == '-'&&message[1] == '.'&&message[2] == '-'&&message[3] == '.') {
			return 'c';
		}
		if (message[0] == '.'&&message[1] == '.'&&message[2] == '-'&&message[3] == '.') {
			return 'f';
		}
		if (message[0] == '.'&&message[1] == '.'&&message[2] == '.'&&message[3] == '.') {
			return 'h';
		}
		if (message[0] == '.'&&message[1] == '-'&&message[2] == '-'&&message[3] == '-') {
			return 'j';
		}
		if (message[0] == '.'&&message[1] == '-'&&message[2] == '.'&&message[3] == '.') {
			return 'l';
		}
		if (message[0] == '.'&&message[1] == '-'&&message[2] == '-'&&message[3] == '.') {
			return 'p';
		}
		if (message[0] == '-'&&message[1] == '-'&&message[2] == '.'&&message[3] == '-') {
			return 'q';
		}
		if (message[0] == '.'&&message[1] == '.'&&message[2] == '.'&&message[3] == '-') {
			return 'v';
		}
		if (message[0] == '-'&&message[1] == '.'&&message[2] == '.'&&message[3] == '-') {
			return 'x';
		}
		if (message[0] == '-'&&message[1] == '.'&&message[2] == '-'&&message[3] == '-') {
			return 'y';
		}
		if (message[0] == '-'&&message[1] == '-'&&message[2] == '.'&&message[3] == '.') {
			return 'z';
		}
	case 5:
		if (message[0] == '.'&&message[1] == '-'&&message[2] == '.'&&message[3] == '-'&&message[4] == '.') {
			return '\0';
		}
	}
	return '?';
}

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

  printf("dupa");

  char data[20] = "cock\r\n";
  HAL_UART_Transmit(&huart2, data, strlen(data), strlen(data)*10);
  char message[1024];
  int len = 0;
  int delta = HAL_GetTick();
  size_t dot_len = 0, space_len = 0;
  _Bool precedency = -1;

  while (1)
  {
	  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
		  ++dot_len;
	  }
	  dot_len /= 4000;
	  char store = 0;
	  if (dot_len < 250) {
		  store = '.';
	  }
	  else {
	  	store = '-';
	  }
	  //send(&huart2, store);
	  message[len++] = store;
	  message[len + 1] = '\0';
	  dot_len = 0;
	  while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
		 space_len++;
		 if (space_len /4000 > 1000) {
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		 }
	  }
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	  space_len /= 4000;
	  if (space_len > 1000) {
		  char res = convert_char(message, len);
		  if (res == '\0') {
			  char eot[] = "\r\nEnd of transmission!\r\n";
			  HAL_UART_Transmit(&huart2, eot, strlen(eot), strlen(eot)*10);
			  break;
		  }
		  send(&huart2, res);
		  len = 0;
		  //send(&huart2, ',');
	  }
  }
  while(1) {

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
