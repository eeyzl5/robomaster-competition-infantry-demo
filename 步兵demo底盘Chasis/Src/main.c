/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "usart.h"
#include "RC.h"
#include "test_imu.h"
#include "mpu6500_reg.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

CRC_HandleTypeDef hcrc;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

osThreadId LED0Handle;
osThreadId LED1Handle;
osThreadId myTask03Handle;
osThreadId PrintNameHandle;
osThreadId updateHandle;
osThreadId WGHandle;
osThreadId IMUHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int set_v, set_spd[4];
uint8_t aTxStartMessage[] = "\r\n****UART-Hyperterminal communication based on IT ****\r\nEnter 10 characters using keyboard :\r\n";
uint8_t aRxBuffer[18]={0x00, 0x04 ,0x20 ,0x00 ,0x01, 0xf8, 0x00 ,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00 };


int CH1 = 3;
int CH2 = 1;
int CH3 = 0;
int CH4 = 2;

int	L_X = 1024;
int	L_Y = 1024;
int	R_X = 1024;
int	R_Y = 1024;

int max_rpm = 2000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI5_Init(void);
void Func_LED0(void const * argument);
void Func_LED1(void const * argument);
void ChassisMotorTask(void const * argument);
void PrintFunc(void const * argument);
void uartTask(void const * argument);
void WGfun(void const * argument);
void IMUfun(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
 // MX_IWDG_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_SPI5_Init();

  /* USER CODE BEGIN 2 */
	IMU_init();
	HAL_GPIO_WritePin(GPIOE, IST_RST_Pin, GPIO_PIN_SET);
	MPU6500_Init();
	IST8310_Init();
	Init_Quaternion();
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_DMA(&huart1,aRxBuffer,18);
//	HAL_UART_Receive_DMA(&huart1,aRxBuffer,18);// ??DMA??
//	HAL_UART_Transmit_DMA(&huart1,aTxStartMessage,sizeof(aTxStartMessage));// DMA????
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of LED0 */
  osThreadDef(LED0, Func_LED0, osPriorityNormal, 0, 128);
  LED0Handle = osThreadCreate(osThread(LED0), NULL);

  /* definition and creation of LED1 */
  osThreadDef(LED1, Func_LED1, osPriorityNormal, 0, 128);
  LED1Handle = osThreadCreate(osThread(LED1), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, ChassisMotorTask, osPriorityHigh, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of PrintName */
  osThreadDef(PrintName, PrintFunc, osPriorityNormal, 0, 128);
  PrintNameHandle = osThreadCreate(osThread(PrintName), NULL);

  /* definition and creation of update */
  osThreadDef(update, uartTask, osPriorityHigh, 0, 128);
  updateHandle = osThreadCreate(osThread(update), NULL);

  /* definition and creation of WG */
  osThreadDef(WG, WGfun, osPriorityRealtime, 0, 128);
  WGHandle = osThreadCreate(osThread(WG), NULL);

  /* definition and creation of IMU */
  osThreadDef(IMU, IMUfun, osPriorityAboveNormal, 0, 128);
  IMUHandle = osThreadCreate(osThread(IMU), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_3TQ;
  hcan1.Init.BS2 = CAN_BS2_5TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN2 init function */
static void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_4TQ;
  hcan2.Init.BS1 = CAN_BS1_3TQ;
  hcan2.Init.BS2 = CAN_BS2_5TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = ENABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 44999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 100000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, IST_INT_Pin|IST_RST_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, NSS_Pin|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : IST_INT_Pin IST_RST_Pin PE7 */
  GPIO_InitStruct.Pin = IST_INT_Pin|IST_RST_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU_INT_Pin */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Pin PF14 */
  GPIO_InitStruct.Pin = NSS_Pin|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* Func_LED0 function */
void Func_LED0(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);	
		osDelay(500);
  }
  /* USER CODE END 5 */ 
}

/* Func_LED1 function */
void Func_LED1(void const * argument)
{
  /* USER CODE BEGIN Func_LED1 */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);		
		osDelay(1000);
  }
  /* USER CODE END Func_LED1 */
}

/* ChassisMotorTask function */
void ChassisMotorTask(void const * argument)
{
  /* USER CODE BEGIN ChassisMotorTask */
  /* Infinite loop */

	my_can_filter_init_recv_all(&hcan1);
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
//	PID_struct_init(&pid_omg, POSITION_PID, 20000, 20000,
//									1.5f,	0.1f,	0.0f	);  //angular rate closeloop.
	HAL_Delay(1000);
	for(int i=0; i<4; i++)
	{
		PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 20000,
									1.5f,	0.1f,	0.0f	);  //4 motos angular rate closeloop.
	}
	HAL_Delay(1000);
	
  /* Infinite loop */
  for(;;)
  {
		
			for(int i=0; i<4; i++)
			{
				pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
			}
			set_moto_current(&hcan1, 	pid_spd[0].pos_out, 
																pid_spd[1].pos_out,
																pid_spd[2].pos_out,
																pid_spd[3].pos_out);
			
			
			/************************
			更新摇杆数据(转换成+-3000rpm)
			*************************/
			//L_X = (RC_CtrlData.rc.lx - 1024)*max_rpm/660; //左摇杆横轴
			//L_Y = (RC_CtrlData.rc.ly - 1024)*max_rpm/660;	//左摇杆纵轴
			L_X = 0;
			L_Y = 0;
			R_X = (RC_CtrlData.rc.rx - 1024)*max_rpm/660;	//右摇杆横轴
			R_Y = (RC_CtrlData.rc.ry - 1024)*max_rpm/660;	//右摇杆纵轴
			
			if(rx_buffer1[1] == 0x01)
			{
				L_X = (RC_CtrlData.rc.lx - 1024)*max_rpm/660; //左摇杆横轴
			}	
			else
			{
				L_X = 0;
			}	
			
			if((L_X<500)&&(L_X>-500)) L_X = 0;
			if((L_Y<500)&&(L_Y>-500)) L_Y = 0;
			if((R_X<500)&&(R_X>-500)) R_X = 0;
			if((R_Y<500)&&(R_Y>-500)) R_Y = 0;
			
			/***********
			更新电机数据
			************/
			set_spd[CH1] = R_Y + L_X + (0-R_X);	  				// CH1 = Thr + Rud - Ail
			set_spd[CH2] = (0-R_Y) + L_X + (0-R_X);				// CH2 = -Thr + Rud - Ail
			set_spd[CH3] = R_Y + L_X + R_X;				// CH3 = Thr + Rud + Ail
			set_spd[CH4] = (0-R_Y) + L_X + R_X;		// CH4 = -Thr + Rud + Ail
			
			osDelay(10);
  }
  /* USER CODE END ChassisMotorTask */
}

/* PrintFunc function */
void PrintFunc(void const * argument)
{
  /* USER CODE BEGIN PrintFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END PrintFunc */
}

/* uartTask function */
void uartTask(void const * argument)
{
  /* USER CODE BEGIN uartTask */
	static int stop=0;//用于判断是否遥控器断开连接
	/******************DMA方式传数据**************************/
  //HAL_UART_Receive_DMA(&huart1,aRxBuffer,18);
	/* Infinite loop */
  for(;;)
  {
		// ??DMA??
		//printf("%x",HAL_UART_Receive_DMA(&huart1,aRxBuffer,18));
		HAL_UART_Receive_DMA(&huart1,aRxBuffer,18);
		RemoteDataProcess(aRxBuffer); 
		//RemoteDataShow();
		//printf("data:%d  \n",RC_CtrlData.rc.ly);
		//printf("Yes: %x\r\n",hdma_usart1_rx.Instance->NDTR); 
		
		/**** 判断是否遥控器断开连接*********/
		stop++;
		if( hdma_usart1_rx.Instance->NDTR <0x12)
		{
			stop=0;
		}
		if(stop>10)
		{
			RC_CtrlData.rc.lx=1024;
			RC_CtrlData.rc.ly=1024;
			RC_CtrlData.rc.rx=1024;
			RC_CtrlData.rc.ry=1024;
		}//{0x00, 0x04 ,0x20 ,0x00 ,0x01, 0xf8, 0x00 ,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00 };

		tx_buffer1[0]=0x00;
		tx_buffer1[1]=RC_CtrlData.rc.s1;
		tx_buffer1[2]=0x01;
		tx_buffer1[3]=90;
		tx_buffer1[4]=RC_CtrlData.rc.s2;
		tx_buffer1[5]=(uint8_t)((uint32_t)(RC_CtrlData.rc.ly-364)*255/1320);//-364*255/(1684-364); //L_Y;
		tx_buffer1[6]=(uint8_t)((uint32_t)(RC_CtrlData.rc.lx-364)*255/1320);
		
		HAL_UART_Transmit_DMA(&huart2,tx_buffer1,sizeof(tx_buffer1));
		huart2.gState = HAL_UART_STATE_READY;
		
		HAL_UART_Receive_DMA(&huart2,rx_buffer1,RX_SIZE);
		
    osDelay(10);
  }
  /* USER CODE END uartTask */
}

/* WGfun function */
void WGfun(void const * argument)
{
  /* USER CODE BEGIN WGfun */
  /* Infinite loop */
  for(;;)
  {
		/************看门狗任务*************/
    osDelay(1);
  }
  /* USER CODE END WGfun */
}

/* IMUfun function */
void IMUfun(void const * argument)
{
  /* USER CODE BEGIN IMUfun */
  /* Infinite loop */
  for(;;)
  {
		/*******IMU 检测********/
		//IMU_getYawPitchRoll(angle);
    osDelay(10);
  }
  /* USER CODE END IMUfun */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */
	if(htim->Instance == TIM3) 
	{
		IMU_Get_Data();
	}
/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
