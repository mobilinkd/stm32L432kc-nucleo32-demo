/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "stdint.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId adcInputTaskHandle;
uint32_t adcInputTaskBuffer[ 512 ];
osStaticThreadDef_t adcInputTaskControlBlock;
osMessageQId adcInputQueueHandle;
uint8_t adcInputQueueBuffer[ 16 * sizeof( uint32_t ) ];
osStaticMessageQDef_t adcInputQueueControlBlock;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ADC_BUFFER_SIZE 176
#define ADC_BLOCKS 8
#define FILTER_SIZE 121

volatile uint32_t adc_counter = 0;
uint32_t adc_buffer[ADC_BUFFER_SIZE];
uint16_t adc_bounce_buffer[ADC_BUFFER_SIZE * ADC_BLOCKS];

const float* filter_taps;
float filter_state[ADC_BUFFER_SIZE + FILTER_SIZE - 1];
float filter_input[ADC_BUFFER_SIZE];
float filter_output[ADC_BUFFER_SIZE];
arm_fir_instance_f32 instance;

const float taps_0dB_121[] = {
  0.00404434702588704,
  -0.0003678805989470367,
  -0.0011037474176397869,
  -0.0023718433790735397,
  -0.004074774206090812,
  -0.005873042355767296,
  -0.007185024927682025,
  -0.00733586918005499,
  -0.005849936137673611,
  -0.0026340821242355635,
  0.001829866887380395,
  0.006596559367932984,
  0.010513363703436297,
  0.012581963716759209,
  0.012278717176141912,
  0.009711068794837135,
  0.005595156551222992,
  0.0010151465722928203,
  -0.0028911100291917724,
  -0.00525632756952279,
  -0.005713225280738633,
  -0.004481173891886628,
  -0.0022979447479362165,
  -0.00020042687909196258,
  0.0007698191454281245,
  -0.00010521900913954391,
  -0.0029591195718788473,
  -0.00722329412770922,
  -0.01171525034180743,
  -0.014940096229612041,
  -0.015533747313789608,
  -0.012735965353880947,
  -0.006719733175529481,
  0.0013488096154956725,
  0.00958058819325866,
  0.015905894530456915,
  0.018729580500272548,
  0.01748781569748245,
  0.012897342590446394,
  0.006795493031300682,
  0.0015881037163025886,
  -0.0005374299552534915,
  0.0016047331197704602,
  0.007673729209166328,
  0.015724242346878387,
  0.022631203832237833,
  0.024935722640487233,
  0.01989507252801227,
  0.00645078432931977,
  -0.014172629267618218,
  -0.038442876378037664,
  -0.06113274585379875,
  -0.07647113797360244,
  -0.07957164384983365,
  -0.06778356575105521,
  -0.04159845430900078,
  -0.0048373906972505945,
  0.03598583922416374,
  0.0730150987796154,
  0.09880455186134979,
  0.10804223448811107,
  0.09880455186134979,
  0.0730150987796154,
  0.03598583922416374,
  -0.0048373906972505945,
  -0.04159845430900078,
  -0.06778356575105521,
  -0.07957164384983365,
  -0.07647113797360244,
  -0.06113274585379875,
  -0.038442876378037664,
  -0.014172629267618218,
  0.006450784329319771,
  0.01989507252801227,
  0.024935722640487233,
  0.022631203832237833,
  0.015724242346878387,
  0.007673729209166332,
  0.0016047331197704602,
  -0.0005374299552534915,
  0.0015881037163025886,
  0.0067954930313006805,
  0.012897342590446394,
  0.017487815697482458,
  0.01872958050027255,
  0.015905894530456915,
  0.00958058819325866,
  0.0013488096154956762,
  -0.006719733175529481,
  -0.012735965353880947,
  -0.015533747313789608,
  -0.014940096229612034,
  -0.01171525034180743,
  -0.00722329412770922,
  -0.0029591195718788473,
  -0.00010521900913954758,
  0.0007698191454281245,
  -0.00020042687909196258,
  -0.0022979447479362165,
  -0.004481173891886626,
  -0.005713225280738633,
  -0.005256327569522788,
  -0.0028911100291917724,
  0.0010151465722928203,
  0.005595156551222992,
  0.009711068794837135,
  0.012278717176141915,
  0.012581963716759209,
  0.010513363703436297,
  0.006596559367932984,
  0.001829866887380395,
  -0.0026340821242355635,
  -0.005849936137673611,
  -0.0073358691800549875,
  -0.007185024927682024,
  -0.005873042355767296,
  -0.004074774206090811,
  -0.0023718433790735397,
  -0.0011037474176397869,
  -0.00036788059894704404,
  0.00404434702588704
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void startAdcInputTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void startADC(void);
float* filter(int16_t* input) __attribute__((section(".bss2")));

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
inline void adc_buffer_copy(int n)
{
  uint16_t* dest = adc_bounce_buffer + (ADC_BUFFER_SIZE * adc_counter);
  uint16_t* src = ((uint16_t*)adc_buffer) + (n * ADC_BUFFER_SIZE);
  memcpy(dest, src, ADC_BUFFER_SIZE * 2);
  adc_counter = ((adc_counter + 1) & (ADC_BLOCKS - 1));
  osMessagePut(adcInputQueueHandle, (uint32_t) dest, 0);
}

// DMA Conversion first half complete.
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* unused)
{
  ADC_STATUS_GPIO_Port->BRR = ADC_STATUS_Pin;
  adc_buffer_copy(0);
}

// DMA Conversion second half complete.
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* unused)
{
  ADC_STATUS_GPIO_Port->BSRR = ADC_STATUS_Pin;
  adc_buffer_copy(1);
}

void startADC(void)
{
  HAL_StatusTypeDef adcStatus = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  if (adcStatus != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc1, adc_buffer, ADC_BUFFER_SIZE * 2);
}

void init_filter(void)
{
  arm_fir_init_f32(&instance, FILTER_SIZE, (float32_t*)filter_taps,
    filter_state, ADC_BUFFER_SIZE);
}

float* filter(int16_t* input)
{
  FIR_STATUS_GPIO_Port->BSRR = FIR_STATUS_Pin;
  for (size_t i = 0; i != ADC_BUFFER_SIZE; i++) {
      filter_input[i] = (input[i] - 2048) * 0.000488281f;
  }
  arm_fir_f32(&instance, filter_input, filter_output, ADC_BUFFER_SIZE);
  FIR_STATUS_GPIO_Port->BRR = FIR_STATUS_Pin;
  return filter_output;

}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

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
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of adcInputTask */
  osThreadStaticDef(adcInputTask, startAdcInputTask, osPriorityNormal, 0, 512, adcInputTaskBuffer, &adcInputTaskControlBlock);
  adcInputTaskHandle = osThreadCreate(osThread(adcInputTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of adcInputQueue */
  osMessageQStaticDef(adcInputQueue, 16, uint32_t, adcInputQueueBuffer, &adcInputQueueControlBlock);
  adcInputQueueHandle = osMessageCreate(osMessageQ(adcInputQueue), NULL);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_MSI, RCC_MCODIV_1);

    /**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration 
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3029;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
     PA8   ------> RCC_MCO
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_STATUS_GPIO_Port, ADC_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|ERROR_Pin|FIR_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5 
                           PA6 PA7 PA9 PA10 
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_STATUS_Pin */
  GPIO_InitStruct.Pin = ADC_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin ERROR_Pin FIR_STATUS_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|ERROR_Pin|FIR_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  }
  /* USER CODE END 5 */ 
}

/* startAdcInputTask function */
void startAdcInputTask(void const * argument)
{
  /* USER CODE BEGIN startAdcInputTask */
  init_filter();
  startADC();
  /* Infinite loop */
  for(;;)
  {
    osEvent result = osMessageGet(adcInputQueueHandle, osWaitForever);
    if (result.status != osEventMessage) continue;
    int16_t* buffer = result.value.p;
    filter(buffer);
  }
  /* USER CODE END startAdcInputTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
