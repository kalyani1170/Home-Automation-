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
#include <stdio.h>
#include <string.h>
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

void LCD_Init(void);
void LCD_Command(unsigned char CMD);
void LCD_Data(unsigned char DAT);
void LCD_StringDisplay(unsigned char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned int adcres;
unsigned char adcstr[4];
unsigned int arr_count;
/* USER CODE END 0 */
unsigned char UART_MSG1[]={"UART communication \r\n"};
unsigned char UART_MSG2[]={"Received Data is "};
unsigned char UART_MSG3[]={"\r\n"};
unsigned char RXBUFFER[20];
unsigned char success;
unsigned char uart_byte_buf[4];
unsigned char password[4]={"1159"};
unsigned char flag=0;
unsigned char count=0;
unsigned int i;
//LCD_StringDisplay("Hello!");
//LCD_StringDisplay("This is Our MiniProject");
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
 LCD_Init();
  /* USER CODE END 2 */
 HAL_UART_Receive_IT(&huart2, uart_byte_buf,4);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0)
//	  	  {
//	  		  arr_count += 100;
//	  		  __HAL_TIM_SET_AUTORELOAD(&htim6,arr_count);
//	  		  HAL_Delay(100);
//	  	  }
//	  	  if(__HAL_TIM_GET_FLAG(&htim6,TIM_FLAG_UPDATE))
//	  	  {
//	  		  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_10);
//	  		  __HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);
//	  	  }

	  	HAL_ADC_Start(&hadc1);
	    HAL_ADC_PollForConversion(&hadc1,100);
	    adcres = HAL_ADC_GetValue(&hadc1);
//	    printf("ADC Readings in Decimal:%d \r\n",adcres);
	    sprintf(adcstr, "%d", adcres);

	    LCD_Command(0x80);
	    LCD_StringDisplay("LDR Reading:");
	    LCD_StringDisplay(adcstr);
	    if (adcres > 2000)
	    {
	    	LCD_Command(0xC0);
	    	LCD_StringDisplay("It's DayTime    ");
	    }
	    else if (adcres < 1800)
	    {
	    	LCD_Command(0xC0);
	    	LCD_StringDisplay("It's Dark      ");
	    }

	    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET)
	    {
	    	LCD_Command (0xC0);
	    	LCD_StringDisplay("Motion Detected  ");
	        if(adcres < 2000)
	        {
	        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);  // Turn on an LED

	        	LCD_Command(0x80);
	        	LCD_StringDisplay("Light is ON     ");
	        	HAL_Delay(5000);
	        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	        	LCD_Command(0x80);
	        	LCD_StringDisplay("Light is Off    ");
	        }
//	        else
//	        {
//	        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);  // Turn off the LED
//	        }
	    }
//	    else
//	    {
//	    	// No motion
//	    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); // Turn off LED
//	    }

	    HAL_Delay(200);
	    if(flag==1)
	    	  {
	    		  LCD_Command(0x80);
	    		  LCD_StringDisplay("Door is Open    ");

	    		  HAL_Delay(500);
	    		  flag=0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1500;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LCD_Init(void)
{
	LCD_Command(0x38);
	HAL_Delay(20);
	LCD_Command(0x06);
	HAL_Delay(20);
    LCD_Command(0x0C);
	HAL_Delay(20);
    LCD_Command(0x01);
	HAL_Delay(20);

}
void LCD_Command(unsigned char CMD)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,(CMD &0x01));
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,(CMD &0x02));
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,(CMD &0x04));
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,(CMD &0x08));
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,(CMD &0x10));
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,(CMD &0x20));
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,(CMD &0x40));
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,(CMD &0x80));
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,1);
	HAL_Delay(3);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
	HAL_Delay(1);
}
void LCD_StringDisplay(unsigned char *str)
{
	while(*str!='\0')
	{
		LCD_Data(*str++);
		HAL_Delay(2);
	}

}

void LCD_Data (unsigned char DAT)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,(DAT & 0x01));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,(DAT & 0x02));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,(DAT & 0x04));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,(DAT & 0x08));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,(DAT & 0x10));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5,(DAT & 0x20));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,(DAT & 0x40));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,(DAT & 0x80));
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,1);
	HAL_Delay(3);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
	HAL_Delay(1);
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//printf("Received Char: \r\n");
	//printf(uart_byte_buf);
	printf("Received password is");
	for (i=0; i<4; i++)
	{
		printf("%c", uart_byte_buf[i]);

	}
	printf("\r\n");
	flag = 0;

	for (i=0; i<4; i++)
	{
		if (uart_byte_buf[i] == password[i])
		{
			flag = 1;
		}
		else
		{
			flag = 0;
		}
	}
	if (flag == 1)
	{
		//LCD_Command(0xC0);
		//LCD_StringDisplay("Door is opened\r\n");
		printf("Door Is Opened \r\n");
		//HAL_Delay(500);
		//flag=1;
	}
	else
	{
		//LCD_Command(0xC0);
		//LCD_StringDisplay("Password mismatch\r\n");
		printf("Incorrect password try again\r\n");

	}
	HAL_UART_Receive_IT(&huart2,uart_byte_buf,4);

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
