/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  char rxbuf[]="waiting\r\n";
  uint8_t txdata[20] = {0};
  unsigned char rxdata[9] = {0};
  uint8_t aRxBuffer[10]={0};
  char PC_txdata[256]={'なんでもいいからおくって\0'};
  short pos_data;



  void pos(unsigned char id,short deg,unsigned short time)
  {
      unsigned char deg_l = (unsigned char)(deg)&0x00FF;
      unsigned char deg_h = (unsigned char)(deg>>8)&0x00FF;
      unsigned char time_l = (unsigned char)(time)&0x00FF;
      unsigned char time_h = (unsigned char)(time>>8)&0x00FF;
      unsigned char sum = (0x09+0x06+id+deg_l+deg_h+time_l+time_h)&0x00FF;

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
      txdata[0]=0x09;   //1 SIZE
      txdata[1]=0x06;   //2 COMMAND
      txdata[2]=0x00;   //3 OPTION
      txdata[3]=id;     //4 ID
      txdata[4]=deg_l;  //5 POS_L
      txdata[5]=deg_h;  //6 POS_H
      txdata[6]=time_l; //7 TIME_L
      txdata[7]=time_h; //8 TIME_H
      txdata[8]=sum;    //9 SUM
      HAL_UART_Transmit(&huart3, txdata, 9, 0xFFFF);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
      HAL_Delay(0.3);
  }
/*    void pos_multi(unsigned char id,short deg,unsigned char id2,short deg2,unsigned short time)
  {
      unsigned char deg_l = (unsigned char)(deg)&0x00FF;
      unsigned char deg_h = (unsigned char)(deg>>8)&0x00FF;
      unsigned char deg2_l = (unsigned char)(deg2)&0x00FF;
      unsigned char deg2_h = (unsigned char)(deg2>>8)&0x00FF;
      unsigned char time_l = (unsigned char)(time)&0x00FF;
      unsigned char time_h = (unsigned char)(time>>8)&0x00FF;
      unsigned char sum = (0x09+0x06+id+deg_l+deg_h+id2+deg2_l+deg2_h+time_l+time_h)&0x00FF;
      txdata[0]=0x09;   //1 SIZE
      txdata[1]=0x06;   //2 COMMAND
      txdata[2]=0x00;   //3 OPTION
      txdata[3]=id;     //4 ID
      txdata[4]=deg_l;  //5 POS_L
      txdata[5]=deg_h;  //6 POS_H
      txdata[6]=id2;     //4 ID2
      txdata[7]=deg2_l;  //5 POS2_L
      txdata[8]=deg2_h;  //6 POS2_H
      txdata[9]=time_l; //7 TIME_L
      txdata[10]=time_h; //8 TIME_H
      txdata[11]=sum;    //9 SUM
      HAL_Delay(0.03);
  }*/

    void speed(unsigned char id,short speed)
    {
        unsigned char speed_l = (unsigned char)(speed)&0x00FF;
        unsigned char speed_h = (unsigned char)(speed>>8)&0x00FF;
        unsigned char sum = (0x09+0x04+0x00+id+speed_l+speed_h+0x30+0x01)&0x00FF;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);

        txdata[0]=0x09;//SIZE
        txdata[1]=0x04;//COMMAND
        txdata[2]=0x00;//OPTION
        txdata[3]=id;//ID
        txdata[4]=speed_l;//SPEED_LOWBYTE
        txdata[5]=speed_h;//SPEED_HIGHBYTE
        txdata[6]=0x30;//ADRESS(SPEED 0x03)
        txdata[7]=0x01;//COUNT
        txdata[8]=sum;//SUM
        HAL_UART_Transmit(&huart3, txdata, 9, 0xFFFF);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
        HAL_Delay(1);
    }

    void write(unsigned char id,unsigned char data,unsigned char adress)
    {
        unsigned char sum = (0x08+0x04+0x00+id+data+adress+0x01)&0x00FF;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
       txdata[0]=0x08;   //1 SIZE
       txdata[1]=0x04;   //2 COMMAND
       txdata[2]=0x00;   //3 OPTION
       txdata[3]=id;     //4 ID
       txdata[4]=data;  //DATA
       txdata[5]=adress;  //ADRESS
       txdata[6]=0x01; //COUNT
       txdata[7]=sum;    //9 SUM
       HAL_UART_Transmit(&huart3, txdata, 8, 0xFFFF);

       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
       HAL_Delay(1);

    }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  write(0xFF,0x00,0x28);
	  write(0xFF,0x01,0x29);

//	  while(1){
//		  HAL_UART_Transmit(&huart2, (uint8_t*)rxbuf,sizeof(rxbuf),0xFFFF);
//		  HAL_UART_Receive(&huart2,rxdata,10,0x100);
//		  write(0xFF,0x00,0x28);
//		  write(0xFF,0x01,0x29);
//		  if(rxdata[0]!=0){
//			  break;
//		  }
//	  }
//	  rxdata[0]=0;

//pos test
	  pos(1,0,2000);
	  HAL_Delay(2000);
	  pos(1,10000,2000);
	  HAL_Delay(2000);

//speed test
      write(0x01,0x04,0x28);
      speed(1,15000);
      HAL_Delay(10000);




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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 1500000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
