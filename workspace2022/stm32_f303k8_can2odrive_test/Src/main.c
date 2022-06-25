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
#include <stdlib.h>
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
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
//CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8]={0};
int set_point=10;
int enc_data;
uint8_t gUartReceived=0;
uint32_t TxMailbox;
uint8_t motor_num = 0x010; //odrive axis node id
uint8_t control_mode = 0x00C;//control mode
//uint8_t control_mode = 0x007; //Set Axis Requested State
//uint8_t control_mode = 0x00A; //get encoder val
//uint8_t control_mode = 0x017; //Get Vbus Voltage

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t uart_data;
char tx_data[]="get data test\r\n";
char tx_data1[]="tx is done\r\n";
char tx_data2[]="\r\n";
uint8_t buffer[256];
int canReceived = 0,canTxHandler=0;
int flag = 1;
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
  MX_CAN_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart2,tx_data,sizeof(tx_data),0xFFFF);

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation=ENABLE;
  sFilterConfig.SlaveStartFilterBank=14;

  if(HAL_CAN_ConfigFilter(&hcan,&sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_CAN_Start(&hcan)!=HAL_OK)
  {
    Error_Handler();
  }

//  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  if(HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
	Error_Handler();
  }

  TxHeader.StdId=(motor_num << 5) + (control_mode);
  TxHeader.RTR = 1;//read data
//  TxHeader.RTR = 1;//pos,state,request input(レシーブいらな�?とき�?
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 0x08;
  TxHeader.TransmitGlobalTime = DISABLE;


  void encoder_read(void){
	  control_mode = 0x00A;
	  TxHeader.RTR = 2 ;//read data
	  TxHeader.StdId=(motor_num << 5) + (control_mode);

	  TxHeader.DLC = 0x08;

	  uint8_t TxData[8];
	  TxData[0] = 0x00;
	  TxData[1] = 0x00;
	  TxData[2] = 0x00;
	  TxData[3] = 0x00;
	  TxData[4] = 0x00;
	  TxData[5] = 0x00;
	  TxData[6] = 0x00;
	  TxData[7] = 0x00;

	  if (HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox) != HAL_OK)
	  {
	     Error_Handler ();
	  }
  }

  void axis_state_change(uint8_t data){
	  control_mode = 0x007;
	  TxHeader.StdId=(motor_num << 5) + (control_mode);
	  TxHeader.RTR = 0;//read data

	  TxHeader.DLC = 0x04;

	  uint8_t TxData[8];
	  TxData[0] = data;
	  TxData[1] = 0x00;
	  TxData[2] = 0x00;
	  TxData[3] = 0x00;
	  TxData[4] = 0x00;
	  TxData[5] = 0x00;
	  TxData[6] = 0x00;
	  TxData[7] = 0x00;

	  if (HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox) != HAL_OK)
	  {
	     Error_Handler ();
	  }
  }

  void set_controll_mode(void){
	  control_mode = 0x00B;
	  TxHeader.StdId=(motor_num << 5) + (control_mode);
	  TxHeader.RTR = 0;//read data

	  TxHeader.DLC = 0x08;

	  uint8_t TxData[8] = {0};
	  TxData[0] = 0x03;
	  TxData[1] = 0x00;
	  TxData[2] = 0x00;
	  TxData[3] = 0x00;
	  TxData[4] = 0x05;
	  TxData[5] = 0x00;
	  TxData[6] = 0x00;
	  TxData[7] = 0x00;
	  for(int i=0;i<8;i++) printf("txdata is %d\r\n",TxData[i]);



	  if (HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox) != HAL_OK)
	  {
	     Error_Handler ();
	  }

  }
  int ctoi(char c) {
  	switch (c) {
  		case '0': return 0;
  		case '1': return 1;
  		case '2': return 2;
  		case '3': return 3;
  		case '4': return 4;
  		case '5': return 5;
  		case '6': return 6;
  		case '7': return 7;
  		case '8': return 8;
  		case '9': return 9;
  		case 'a': return 0xf;
  		case 'b': return 0xb;
  		case 'c': return 0xc;
  		case 'd': return 0xd;
  		case 'e': return 0xe;
  		case 'f': return 0xf;
  		default: return 0;
  	}
  }

  int ftox(float angle){
	  union { float x; int i; } a;
	  a.x = angle;
	  char test[32];
	  sprintf(test,"0x%08x\n", a.i);
	  printf("test %s\r\n",test);
	  int data =0;
	  data=test[9]-'0';
	  data+=ctoi(test[8])<<4;
	  data+=ctoi(test[7])<<8;
	  data+=ctoi(test[6])<<12;
	  data+=ctoi(test[5])<<16;
	  data+=ctoi(test[4])<<20;
	  data+=ctoi(test[3])<<24;
	  data+=ctoi(test[2])<<28;
	  printf("data %08x\r\n",data);

	  return data;
  }

  void pos_change(float data){
	  control_mode = 0x00C;
	  TxHeader.StdId=(motor_num << 5) + (control_mode);
	  TxHeader.RTR = 0;//read data

	  TxHeader.DLC = 0x08;

	  int set_pos = ftox(data);
	  uint8_t TxData[8] = {0};
	  TxData[0] = set_pos & 0xFF;
	  TxData[1] = (set_pos>>8)&0xFF ;
	  TxData[2] = (set_pos>>16)&0xFF;
	  TxData[3] = (set_pos>>24)&0xFF;
	  TxData[4] = 0xFF;
	  TxData[5] = 0xFF;
	  TxData[6] = 0xFF;
	  TxData[7] = 0xFF;

//	  TxData[0] = 0x00;
//	  TxData[1] = 0x00;
//	  TxData[2] = 0xA0;
//	  TxData[3] = 0x40;
//	  TxData[4] = 0x00;
//	  TxData[5] = 0x00;
//	  TxData[6] = 0x00;
//	  TxData[7] = 0x00;


	  if (HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox) != HAL_OK)
	  {
	     Error_Handler ();
	  }
  }

  HAL_UART_Receive_IT(&huart2, buffer,1);

//  axis_state_change(0x08);
//  HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	printf("------mode select---------\r\n");
	printf("encoder read: 1\r\n");
	printf("CLOSED_LOOP : 2\r\n");
	printf("IDLE        : 3\r\n");
	printf("pos mode    : 4\r\n");
	while(gUartReceived==0){
	}

	int mode = buffer[0];
	gUartReceived=0;

	switch(mode){
	case 49:
		printf("encoder reading... 1\r\n");
		while(gUartReceived==0){
			encoder_read();
			HAL_Delay(10);
		}
		gUartReceived=0;
		break;


	case 50:
		printf("Closed loop\r\n");
		axis_state_change(0x08);
		break;

	case 51:
		printf("IDLE\r\n");
		axis_state_change(0x01);
		break;

	case 52:
		printf("pos select : 0 ~ 6\r\n");
		printf("quit : press q  \r\n");
//		set_controll_mode();

		while(1){
			while(gUartReceived==0){
//				pos_change(10000);
//				HAL_Delay(10);
			}
			int pos = buffer[0]-'0';
			printf("pos is : %d\r\n",pos);
			gUartReceived=0;
			if(buffer[0]=='q') break;

			pos_change(pos);
		}

	default:
		printf("Err: this mode is not found : %d\r\n",mode);

	}


//	  if(gUartReceived){
//		  printf("mailbox data :  %d\r\n",HAL_CAN_GetTxMailboxesFreeLevel(&hcan));
//		  if(buffer[0]=='1'){
//			  axis_state_change(0x08);
//			  printf("Closed loop\r\n");
//		  }
//		  else if(buffer[0]=='2'){
//			  axis_state_change(0x01);
//			  printf("IDLE\r\n");
//		  }else if(buffer[0]=='3'){
//			  pos_change(0x00);
//		  }
//		  gUartReceived=0;
//		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
//		  HAL_Delay(100);
//		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
//	  }
//	  printf("mailbox data :  %d\r\n",HAL_CAN_GetTxMailboxesFreeLevel(&hcan));
//	  encoder_read();


//	  printf("mailbox data :  %d\r\n",HAL_CAN_GetTxMailboxesFreeLevel(&hcan));
//	  for(int i=0;i<100;i++){
//		  encoder_read();
//		  HAL_Delay(10);
//	  }
//	  HAL_Delay(1000);

//	  pos_change();
//	  for(int i=0;i<20;i++){
//		printf("mailbox data :  %d\r\n",HAL_CAN_GetTxMailboxesFreeLevel(&hcan));
//		  pos_change();
//		  axis_state_change();
//		  HAL_Delay(10);
//	  }

//	  if (HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox) != HAL_OK)
//	  {
//	     Error_Handler ();
//	  }



//	  HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);

//	  while(canTxHandler==0){
//		  HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
//		  printf("test %d\r\n",HAL_CAN_GetTxMailboxesFreeLevel(&hcan));
//	  }
//	  canTxHandler=0;
//	  HAL_Delay(5000);
//	  canTxHandler=0;
//
//	  while(canTxHandler==0){
//		  TxData[0] = 0x1;
//	  	  HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
//	  	  printf("test3 %d\r\n",HAL_CAN_GetTxMailboxesFreeLevel(&hcan));
//	  }
//	  HAL_Delay(5000);



//	HAL_UART_Transmit(&huart2,tx_data,sizeof(tx_data),0xFFFF);

	//	for(int j=0; j< 4; j++){
//		HAL_UART_Transmit(&huart2,&RxData[j],1,0xFFFF);
//
//		HAL_Delay(10);
//	}
//	HAL_UART_Transmit(&huart2,tx_data2,sizeof(tx_data2),0xFFFF);
//	HAL_Delay(10);
//	//HAL_UART_Transmit(&huart2,RxData,8,0xFFFF);
//	for(int j=4; j< 8; j++){
//		HAL_UART_Transmit(&huart2,&RxData[j],1,0xFFFF);
//		HAL_Delay(10);
//	}
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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	gUartReceived = 1;
	HAL_UART_Receive_IT(&huart2, buffer,1);
}

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,50);
  return len;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	canTxHandler = 1;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
	printf("tx done \r\n");
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_)
{
  CAN_RxHeaderTypeDef RxHeader;
  canReceived = 1;
  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
  if (RxData[0]!=0){
	enc_data=RxData[4] & 0xFF;
	enc_data+=(RxData[5] & 0xFF) <<8;
	enc_data+=(RxData[6] & 0xFF)<<16;
	enc_data+=(RxData[7] & 0xFF)<<24;
	printf("enc_data : %d\r\n",enc_data);
  }

//  if(HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
//  {
//	Error_Handler();
//  }


  flag *= -1;
  if(flag >0){
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
  }else{
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
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
		  HAL_UART_Transmit(&huart2,tx_data,sizeof(tx_data),0xFFFF);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
