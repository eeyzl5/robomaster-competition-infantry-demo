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
#include "test_imu.h"
#include "mpu6500_reg.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

WWDG_HandleTypeDef hwwdg;

osThreadId LED0Handle;
osThreadId updateHandle;
osThreadId myTask03Handle;
osThreadId PrintNameHandle;
osThreadId WGHandle;
osThreadId IMUHandle;
osThreadId ShootingHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define setbit(x,y) x|=(1<<y) //将X的第Y位置1
#define clrbit(x,y) x&=~(1<<y) //将X的第Y位清0
//PID_TypeDef motor_pid[4];
PID_TypeDef could_pid;
PID_TypeDef moto_pid;
uint8_t state_flag=0; 
int speed_step_sign = +1;
char InfoBuffer[1000];
uint16_t TIM_COUNT[2];

#define SpeedStep 500
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM3_Init(void);
static void MX_WWDG_Init(void);
void uartTask(void const * argument);
void updatefun(void const * argument);
void yawTask(void const * argument);
void PrintFunc(void const * argument);
void WGFun(void const * argument);
void IMUFun(void const * argument);
void pitchTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint32_t map(uint32_t val, uint32_t I_Min, uint32_t I_Max, uint32_t O_Min, uint32_t O_Max){
    return(val/(I_Max-I_Min)*(O_Max-O_Min) + O_Min);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_PWR_PVDCallback(void)
{

}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
/*这代码有很多需要改的地方，不如云台yaw轴控制在偏移小角度的时候一卡一卡的，或许可以通过调PID来改善。舵机的控制也不是很好，不都到时候这个是要换成6623的*/

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
  MX_CRC_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_SPI5_Init();
  MX_TIM3_Init();
 // MX_WWDG_Init();

  /* USER CODE BEGIN 2 */
	//HAL_WWDG_Start_IT(&hwwdg);
	
	IMU_init();
	HAL_GPIO_WritePin(GPIOE, IST_RST_Pin, GPIO_PIN_SET);
	MPU6500_Init();
	IST8310_Init();
	Init_Quaternion();
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	
	HAL_Delay(1000);
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,0x3C*9999/255);
	HAL_Delay(3000);
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,0x3C*9999/255);
	HAL_Delay(3000);

	my_can_filter_init_recv_all(&hcan1);     //配置CAN过滤器
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);   //启动CAN接收中断

  pid_init(&moto_pid);
  moto_pid.f_param_init(&moto_pid,PID_Speed,16384,5000,10,0,8000,0,1.5f,0.1f,0);
    
	pid_init(&could_pid);
  could_pid.f_param_init(&could_pid,PID_Position,5000,5000,10,1,5000,3500, 0.2f,0.0f,1.2f);
  //__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); 
	//HAL_WWDG_Start_IT(&hwwdg);
	//MX_WWDG_Init();
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
  osThreadDef(LED0, uartTask, osPriorityNormal, 0, 128);
  LED0Handle = osThreadCreate(osThread(LED0), NULL);

  /* definition and creation of update */
  osThreadDef(update, updatefun, osPriorityNormal, 0, 128);
  updateHandle = osThreadCreate(osThread(update), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, yawTask, osPriorityHigh, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of PrintName */
  osThreadDef(PrintName, PrintFunc, osPriorityAboveNormal, 0, 128);
  PrintNameHandle = osThreadCreate(osThread(PrintName), NULL);

  /* definition and creation of WG */
  osThreadDef(WG, WGFun, osPriorityHigh, 0, 128);
  WGHandle = osThreadCreate(osThread(WG), NULL);

  /* definition and creation of IMU */
  osThreadDef(IMU, IMUFun, osPriorityIdle, 0, 128);
  IMUHandle = osThreadCreate(osThread(IMU), NULL);

  /* definition and creation of Shooting */
  osThreadDef(Shooting, pitchTask, osPriorityIdle, 0, 128);
  ShootingHandle = osThreadCreate(osThread(Shooting), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	//  MX_WWDG_Init();
  /* USER CODE END RTOS_QUEUES */


  /* Start scheduler */
  osKernelStart();
	could_pid.target = 3500;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_9TQ;
  hcan1.Init.BS2 = CAN_BS2_4TQ;
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
  htim3.Init.Period = 42000;
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

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 41;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 9999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim5);

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

/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 80;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
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
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

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
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 10, 0);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, IST_INT_Pin|IST_RST_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, NSS_Pin|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* uartTask function */
void uartTask(void const * argument)
{
	
  /* USER CODE BEGIN 5 */
	tx_buffer[0] = 0x00;
  /* Infinite loop */
  for(;;)
  {
	//	vTaskList(InfoBuffer);

		//接收
		HAL_UART_Receive_DMA(&huart2,rx_buffer,RX_SIZE);
		
		//发送
		HAL_UART_Transmit_DMA(&huart2,tx_buffer,sizeof(tx_buffer));
		huart2.gState = HAL_UART_STATE_READY;
		
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
		
		osDelay(10);
  }
  /* USER CODE END 5 */ 
}

/* updatefun function */
void updatefun(void const * argument)
{
	
  /* USER CODE BEGIN updatefun */
	
  /* Infinite loop */
  for(;;)
  {

		if(rx_buffer[0]==0x00)
		{
			
			HAL_UART_Transmit_DMA(&huart6,rx_buffer,RX_SIZE);
			huart6.gState = HAL_UART_STATE_READY;

			/**********************，摩擦轮****************************/
			if(rx_buffer[4]==0x02)
			{
				__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,0x40*9999/255);
				__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,0x40*9999/255);
			}else if(rx_buffer[4]==0x01)
			{
				__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,0x3E*9999/255);
				__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,0x3E*9999/255);
			}else
			{
				__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,0x3C*9999/255);
				__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,0x3C*9999/255);
			}	
			/***********************拨弹电机***************************/
			if(rx_buffer[1]==0x01)
			{
				 moto_pid.target = 2000;
			}
			else if(rx_buffer[1]==0x02)
			{
				 moto_pid.target = 4000;
			}
			else
			{
				 moto_pid.target = 0;
			}
			/************激光******************/
			if(rx_buffer[1]&0x01)
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		}

		moto_pid.f_cal_pid(&moto_pid,moto_chassis[0].speed_rpm);    //根据设定值进行PID计算。		
    set_moto_current(&hcan1,moto_pid.output,0,0,0);//将PID的计算结果通过CAN发送到电机
		
    osDelay(10);
  }
  /* USER CODE END updatefun */
}

/* pitchTask function */
void pitchTask(void const * argument)
{
  /* USER CODE BEGIN pitchTask */
	static float pitch = 3500;
	static int pitch_max = 3800;
	static int pitch_min = 3100;
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,pitch);
  /* Infinite loop */
  for(;;)
  {
		/*********************舵机****************************/
			if((rx_buffer[5]>0x7f)||(rx_buffer[5]<0x7f))
			{

				pitch -= ((rx_buffer[5]-0x7f))*0.3;
				if(pitch > pitch_max)
				{
					pitch = pitch_max;
				}	
				if(pitch < pitch_min)
				{
					pitch = pitch_min;
				}	
				__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,pitch);
				
			}
    osDelay(10);
  }
  /* USER CODE END pitchTask */
}

/* yawTask function */
void yawTask(void const * argument)
{
  /* USER CODE BEGIN yawTask */
  /* Infinite loop */
	static float update_yaw=3500;
	static int lx = 0;
	could_pid.target = update_yaw;
	
	for(;;)
	{
   		/********************云台电机******************************/
		tx_buffer[1] = 0x00;
		
			if((rx_buffer[6]>0x7f)||(rx_buffer[6]<0x7f))
			{
				lx = rx_buffer[6]-0x7f;
				
				if(lx > 2)
				{
					lx = 2;
				}
				if(lx < -2)
				{
					lx = -2;
				}
				
				update_yaw -= lx;

				if(update_yaw>=5000)
				{
					update_yaw=5000;
					tx_buffer[1] = 0x01;
				}
				
				if(update_yaw<=1000)
				{
					update_yaw=1000;
					tx_buffer[1] = 0x01;
				}
		
				could_pid.target = update_yaw;//+overflag*8191;

			}
			
    could_pid.f_cal_pid(&could_pid,overflag*8191+cloud[0].angle); 
		set_could_current(&hcan1,0,0,could_pid.output,0);
		
    osDelay(1);      //PID控制频率1000HZ
  }
  /* USER CODE END yawTask */
}

/* PrintFunc function */
void PrintFunc(void const * argument)
{
  /* USER CODE BEGIN PrintFunc */
//	HAL_UART_Receive_DMA(&huart2,rx_buffer,RX_SIZE);
  /* Infinite loop */
  for(;;)
  {

		//	HAL_UART_Transmit_DMA(&huart2,rx_buffer,TX_SIZE);
		//	huart2.gState = HAL_UART_STATE_READY;
			//HAL_UART_Transmit_DMA(&huart2,rx_buffer,TX_SIZE);
			//huart2.gState = HAL_UART_STATE_READY;
		
		//printf("%x",rx_buffer[1]);
		//			huart6.gState = HAL_UART_STATE_READY;

		//printf(rx_buffer);
		//printf("imu_data.mx: %d  ",imu_data.mx);
	//	printf("imu_data.my: %d\n",imu_data.my);

		//printf("Y: %.1f			P: %.1f			R: %.1f\r\n",yaw_angle,pitch_angle,roll_angle);
			osDelay(100);
  }
  /* USER CODE END PrintFunc */
}

/* WGFun function */
void WGFun(void const * argument)
{
  /* USER CODE BEGIN WGFun */
  /* Infinite loop */
  for(;;)
  {
		/*****************************请在这里加入看门狗**************************************/
		osDelay(40);
//		HAL_WWDG_Refresh(&hwwdg);
  }
  /* USER CODE END WGFun */
}

/* IMUFun function */
void IMUFun(void const * argument)
{
  /* USER CODE BEGIN IMUFun */
  /* Infinite loop */
  for(;;)
  {
/*******************其实云台应该有一个专门的IMU以它的角度来控制云台而不是用机械角*************************/
		//IMU_getYawPitchRoll(angle);
    osDelay(10);
  }
  /* USER CODE END IMUFun */
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
  if (htim->Instance == TIM4) 
	{
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

	if (htim->Instance == TIM3) 
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
