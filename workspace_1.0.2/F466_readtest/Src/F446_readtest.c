/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  uint8_t txdata[9] = {0};
  uint8_t txdata2[7] = {0};
  uint8_t rxdata[9] = {0};
  char txbuf[] ="\n\r hello world";
  int pos_data;

    void pos(unsigned char id,short deg,unsigned short time)
    {
      unsigned char deg_l = (unsigned char)(deg)&0x00FF;
      unsigned char deg_h = (unsigned char)(deg>>8)&0x00FF;
      unsigned char time_l = (unsigned char)(time)&0x00FF;
      unsigned char time_h = (unsigned char)(time>>8)&0x00FF;
      unsigned char sum = (0x09+0x06+id+deg_l+deg_h+time_l+time_h)&0x00FF;

      txdata[0]=0x09;   //1 SIZE
      txdata[1]=0x06;   //2 COMMAND
      txdata[2]=0x00;   //3 OPTION
      txdata[3]=id;     //4 ID
      txdata[4]=deg_l;  //5 POS_L
      txdata[5]=deg_h;  //6 POS_H
      txdata[6]=time_l; //7 TIME_L
      txdata[7]=time_h; //8 TIME_H
      txdata[8]=sum;    //9 SUM
      HAL_Delay(0.03);

    }
    void current(unsigned char id,short cur,unsigned short time)
       {
         unsigned char cur_l = (unsigned char)(cur)&0x00FF;
         unsigned char cur_h = (unsigned char)(cur>>8)&0x00FF;
         unsigned char time_l = (unsigned char)(time)&0x00FF;
         unsigned char time_h = (unsigned char)(time>>8)&0x00FF;
         unsigned char sum = (0x09+0x06+id+cur_l+cur_h+time_l+time_h)&0x00FF;

         txdata[0]=0x09;   //1 SIZE
         txdata[1]=0x06;   //2 COMMAND
         txdata[2]=0x00;   //3 OPTION
         txdata[3]=id;     //4 ID
         txdata[4]=cur_l;  //5 POS_L
         txdata[5]=cur_h;  //6 POS_H
         txdata[6]=time_l; //7 TIME_L
         txdata[7]=time_h; //8 TIME_H
         txdata[8]=sum;    //9 SUM
         HAL_Delay(0.03);

       }

    void read(unsigned char id,unsigned char address,unsigned char length)
    {
        unsigned char sum = (0x07+0x03+0x00+id+address+length)&0x00FF;
        txdata2[0]=0x07;       //SIZE
        txdata2[1]=0x03;       //COMMAND
        txdata2[2]=0x00;       //OPTION
        txdata2[3]=id;         //ID
        txdata2[4]=address;    //ADDRESS(Current 0x48,MotorTemperature 0x46)
        txdata2[5]=length;     //LENGTH byte
        txdata2[6]=sum;        //SUM
        //wait(0.001);
    }


    void write(unsigned char id,unsigned char data)
    {
        unsigned char sum = (0x08+0x04+0x00+id+data+0x28+0x01)&0x00FF;
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
       txdata[0]=0x08;   //1 SIZE
       txdata[1]=0x04;   //2 COMMAND
       txdata[2]=0x00;   //3 OPTION
       txdata[3]=id;     //4 ID
       txdata[4]=data;  //DATA
       txdata[5]=0x28;  //ADRESS
       txdata[6]=0x01; //COUNT
       txdata[7]=sum;    //9 SUM
       HAL_UART_Transmit(&huart6, txdata, 8, 0xFFFF);
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
       HAL_Delay(0.3);
    }


    void speed(unsigned char id,short speed)
    {
        unsigned char speed_l = (unsigned char)(speed)&0x00FF;
        unsigned char speed_h = (unsigned char)(speed>>8)&0x00FF;
        unsigned char sum = (0x09+0x04+0x00+id+speed_l+speed_h+0x30+0x01)&0x00FF;

        txdata[0]=0x09;//SIZE
        txdata[0]=0x04;//COMMAND
        txdata[0]=0x00;//OPTION
        txdata[0]=id;//ID
        txdata[0]=speed_l;//SPEED_LOWBYTE
        txdata[0]=speed_h;//SPEED_HIGHBYTE
        txdata[0]=0x30;//ADRESS(SPEED 0x03)
        txdata[0]=0x01;//COUNT
        txdata[0]=sum;//SUM
        HAL_Delay(0.3);
    }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //PCへの送信テスト
	  	  HAL_UART_Transmit(&huart2,(uint8_t*) txbuf,sizeof(txbuf),0xFFFF);
	        HAL_Delay(500);

	  //FreeモードからNormalモードへ
	    //    write(0xFF,0x00);
	    //    pos(0x01,0,1000);

	  //B3Mの現在位置を確認


	  	  while(1)
	  	  {
	  		   //B3Mから情報を読み取る
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	  		  read(0x08,0x2C,0x02);
	  		  HAL_UART_Transmit(&huart6, txdata2,7,0xFFFF);
	  		  //HAL_Delay(0.3);
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	  		  HAL_UART_Receive(&huart6,rxdata,10,0x100);


	  	  if(rxdata[1]!=0)
	  	  	  {
	  		  break;
	  	  	  }
	  	  }

	  	  pos_data=rxdata[4];
	  	  pos_data+=rxdata[5]<<8;
//	  	  pos_data+=rxdata[6]<<16;
//	  	  pos_data+=rxdata[7]<<24;
	  	  rxdata[1]=0;

	  //B3Mの情報をPCへ送信
	  	  char PC_txdata[256]={'\0'};
	  		  sprintf(PC_txdata,"%d\n", pos_data); // 文字列に変換
	  		  HAL_UART_Transmit(&huart2, (uint8_t*)PC_txdata,sizeof(PC_txdata),0xFFFF);
	  		  HAL_Delay(100);

/*	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	  		  pos(0x03,9000,1000);
	  		  HAL_UART_Transmit(&huart6, txdata2,9,0xFFFF);
	  		  //HAL_Delay(100);
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);*/


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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
