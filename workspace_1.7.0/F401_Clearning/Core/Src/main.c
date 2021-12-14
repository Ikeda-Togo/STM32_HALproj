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
#include <stdio.h>
#include <stdbool.h>
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t stop;

void delay_us (uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
    {
//      HAL_GPIO_WritePin(GPIOA, DIR_Pin, 1);
      printf("off\r\n");
      stop=0;
    }
    else if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
    {
//      HAL_GPIO_WritePin(GPIOA, DIR_Pin, 0);
      printf("on\r\n");
      stop=1;
    }
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
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  printf("start\r\n");
  uint8_t txdata[9] = {0};
  short duty=10;
  int out_range_count = 0;
  int in_range_count = 0;
  bool clearning_handler= false;

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim2);
//  HAL_TIM_Base_Start(&htim2);

  void init_motion(){
	  printf("init motion...\r\n");
	  HAL_GPIO_WritePin(EN_step_GPIO_Port, EN_step_Pin, 0);
	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	  {
	    HAL_GPIO_WritePin(GPIOA, DIR_Pin, 0);
	    while(1){
		  HAL_GPIO_WritePin(GPIOA, Step_Pin, 1);
//		  HAL_Delay(0.4);
		  delay_us(800);
		  HAL_GPIO_WritePin(GPIOA, Step_Pin, 0);
//		  HAL_Delay(0.01);
		  delay_us(200);
		  if(stop == 1){
			  break;
		  }
	    }
	  }
	  HAL_GPIO_WritePin(GPIOA, DIR_Pin, 1);
	  for(int i=0;i<200;i++){
		  HAL_GPIO_WritePin(GPIOA, Step_Pin, 1);
	  	  HAL_Delay(1);
	  	  HAL_GPIO_WritePin(GPIOA, Step_Pin, 0);
	  	  HAL_Delay(1);
	  }
	  HAL_GPIO_WritePin(EN_step_GPIO_Port, EN_step_Pin, 1);


  }

  int sonic_sense(){
	  GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Pin = GPIO_PIN_1;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
	  delay_us(2);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
	  delay_us(5);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);

	  GPIO_InitStruct.Pin = GPIO_PIN_1;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // digital Input
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1) == GPIO_PIN_RESET);

	  __HAL_TIM_SET_COUNTER(&htim2,0);
	  while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1) == GPIO_PIN_SET);
	  int counter = __HAL_TIM_GET_COUNTER(&htim2);
	  delay_us(500);

	  return(counter);

  }

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


  	  HAL_UART_Transmit(&huart6, txdata, 9, 0xFFFF);
      HAL_Delay(0.03);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
  }
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

        HAL_UART_Transmit(&huart6, txdata, 9, 0xFFFF);
        HAL_Delay(0.03);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
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

       HAL_UART_Transmit(&huart6, txdata, 8, 0xFFFF);
       HAL_Delay(0.03);
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);

    }

    void alcohol_motion(){
  	  write(0x04,0x00,0x28);
  	  write(0x04,0x01,0x29);
  	  //水平に調整
  	  pos(0x04,0,1000);
  	  HAL_Delay(1000);

  	  //右のアクリル噴�?
  	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,12);
  	  HAL_Delay(500);
  	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,22);
  	  HAL_Delay(500);
  	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,12);
  	  HAL_Delay(1000);

  	  //左アクリルの噴�?
  	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,24);
  	  HAL_Delay(500);
  	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,22);
  	  HAL_Delay(500);
  	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,12);
  	  HAL_Delay(1000);

  	  //机に噴�?
  	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,18);
  	  HAL_Delay(500);
  	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,22);
  	  HAL_Delay(500);
  	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,12);
  	  HAL_Delay(1000);

  	  //中央アクリルに噴�?
  	  pos(0x04,6000,1000);
  	  HAL_Delay(1000);
  	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,22);
  	  HAL_Delay(500);
  	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,12);

  	  //初期位置に戻�?
  	  pos(0x04,-9000,1000);
  	  HAL_Delay(1000);


    }

    void clearning_motion(){

      printf("clearning motion...\r\n");
      write(0x01,0x04,0x28);
      write(0x02,0x04,0x28);
      write(0x03,0x04,0x28);
      write(0xFF,0x01,0x29);

      //モップ回転
   	  speed(0x01,15000);
   	  speed(0x02,15000);
   	  speed(0x03,-15000);

  	  HAL_Delay(1000);

  	  HAL_GPIO_WritePin(EN_step_GPIO_Port, EN_step_Pin, 0);
  	  HAL_GPIO_WritePin(GPIOA, DIR_Pin, 1);

  	  //??��?��?復ス??��?��???��?��?プ数
  	  for(int i=0;i<10000;i++){
  		  HAL_GPIO_WritePin(GPIOA, Step_Pin, 1);
  		  delay_us(400);
  	  	  HAL_GPIO_WritePin(GPIOA, Step_Pin, 0);
  	  	  delay_us(400);
  	  }
  	  HAL_GPIO_WritePin(GPIOA, DIR_Pin, 0);
  	  for(int i=0;i<10000;i++){
  		  HAL_GPIO_WritePin(GPIOA, Step_Pin, 1);
  		  delay_us(400);
  	  	  HAL_GPIO_WritePin(GPIOA, Step_Pin, 0);
  	  	  delay_us(400);
  	  	  if (stop ==1){
  	  		  break;
  	  	  }
  	  }
  	  HAL_GPIO_WritePin(EN_step_GPIO_Port, EN_step_Pin, 1);

  	  //回転ストップ
  	  speed(0xFF,0);

  	  //机ふきふき
  	  duty=12;
  	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,duty);
  	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,30-duty);
  	  HAL_Delay(1000);

  	  duty=21;
  	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,duty);
  	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,30-duty);
  	  HAL_Delay(1000);

    }

//  	  write(0xFF,0x04,0x28);
//  	  write(0xFF,0x01,0x29);
//  	  speed(0xFF,15000);

//  	init_motion();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  alcohol_motion();


/*------------------sonic_sense test------------------------*/

//	  while(1){
//		  int distance = sonic_sense();
//		  printf("distance=%d\r\n",distance);
//		  HAL_Delay(500);
//
//		  if(distance>5000){         //??��?��?な??��?��?時間の計測
//			  out_range_count++;
//			  in_range_count=0;
//		  }
//		  else{                     //??��?��?る時間�???��?��計測
//			  out_range_count=0;
//			  in_range_count++;
//			  if (in_range_count>10){ //滞在時間の設??��?��?
//				  clearning_handler= true;
//			  }
//		  }
//
//		  if(clearning_handler== true && out_range_count > 5){ //??��?��?な??��?��?時間の設??��?��?
//			  clearning_motion();
//			  init_motion();
//			  clearning_handler= false;
//		  }
//	  }


/*-----------------------Servo test---------------------------*/

//	  duty=12;
//	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,duty);
//	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,30-duty);
//	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,duty);
//	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,30-duty);
//	  HAL_Delay(1000);
//
//	  duty=21;
//	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,duty);
//	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,30-duty);
//	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,duty);
//	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,30-duty);
//	  HAL_Delay(1000);

/*-----------------------stepping test-------------------------*/

//	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
//	  {
//	    HAL_GPIO_WritePin(GPIOA, DIR_Pin, 1);
//
//	    while(1){
//	    	HAL_GPIO_WritePin(GPIOA, Step_Pin, 1);
//	    	HAL_Delay(4);
//	    	HAL_GPIO_WritePin(GPIOA, Step_Pin, 0);
//	    	HAL_Delay(1);
//	    	if(stop == 1){
//			  break;
//		  }
//	    }
//	  }
//	  if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
//	  {
//	    HAL_GPIO_WritePin(GPIOA, DIR_Pin, 0);
//		  for (int i=0; i<100; i++) {
//			  HAL_GPIO_WritePin(GPIOA, Step_Pin, 1);
//			  HAL_Delay(4);
//			  HAL_GPIO_WritePin(GPIOA, Step_Pin, 0);
//			  HAL_Delay(1);
//		  }
//	  }

/*-----------------B3M test-----------------------*/
//	  write(0xFF,0x00,0x28);
//	  write(0xFF,0x01,0x29);
//  	  pos(0x04,9000,1000);
//  	  HAL_Delay(1000);
//  	  pos(0x04,0,1000);
//  	  HAL_Delay(1000);
//  	  pos(0x04,-9000,1000);
//  	  HAL_Delay(1000);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 625;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart6.Init.BaudRate = 1500000;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin|Step_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_step_GPIO_Port, EN_step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin Step_Pin DIR_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin|Step_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_step_Pin */
  GPIO_InitStruct.Pin = EN_step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_step_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
  return len;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
