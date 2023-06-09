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
#include "ModBusRTU.h"
#include <stdint.h>
#include "arm_math.h"
#include <math.h>
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
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
ModbusHandleTypedef hmodbus;

typedef union{
	int16_t U16;
	int8_t U8;
}int16int8;
int16int8 registerFrame[70];


// Modbus function
uint8_t mainstate;
typedef enum
{
	checkstate, SetPoint, Homing, SetPick_PlaceTray, Run_tray_mode
}BaseStatus;


// Joystick controller
typedef struct{
	uint16_t X_Axis;
	uint16_t Y_Axis;
	uint16_t joy_button;
}All_ADC;

All_ADC adc;

uint8_t left, right; //X_axis
uint8_t top, bottom; //Y_axis



//define pick place mode

uint8_t pick_or_place; // to see this is in pick or place mode /0/ = pick , /1/ = place


typedef struct{
	float X_point[3];
	float Y_point[3];
}point_array;

point_array pick_point,place_point;








// define position velocity accerelation

int32_t QEIRead;
float position, velocity, accerelation;


float velocity_array[10];
uint8_t velocity_index = 0;

float avg_velocity, avg_accerelation;


int32_t degree; // use in trajectory function





// trajectory mode

float sampling_time, speed_happen, present_position, time_final;
uint8_t toggle_trajec = 1;

uint8_t arrived = 1;
float pick_place_array[10];






// motor control

float Vfeedback, error_hap;





// calculate tray position

typedef enum
{
	plus_plus,   //   X /+/, Y /+/
	minus_plus,  //   X /-/, Y /+/
	minus_minus, //   X /-/, Y /-/
	plus_minus   //   X /+/, Y /-/

}quadrant;

float32_t degree_triangle;
float32_t payback_val;


// undefine or debug

uint8_t debug_1;
uint16_t debug_2;
float debug_3;

// I2C end effector variable
uint8_t ENEstatus; // for reading status from end effector as 8 bit MSB
int modeSelect; // select mode 1 - 9 in live expression
uint8_t slaveAddress = 0x15;
//Sequences modes
uint8_t softResetseq[4] = {0x00, 0xFF,0x55,0xAA};
uint8_t Emergencyseq[1] = {0xF1};
uint8_t stopEmergencyseq[4] = {0xE5, 0x7A, 0xFF, 0x81};
uint8_t startTestseq[2] = {0x01, 0x02};
uint8_t stopTestseq[2] = {0x01, 0x00};
uint8_t startRunseq[2] = {0x10, 0x13};
uint8_t stopRunseq[2] = {0x10, 0x8C};
uint8_t pickseq[2] = {0x10, 0x5A};
uint8_t placeseq[2] = {0x10, 0x69};

 uint8_t statusTemp[1];
 uint8_t readFlag = 0;
 uint8_t readFlag2 = 0;
 uint8_t read_count = 0;



// home status
uint8_t home_in = 1;
uint8_t flip_position = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void heartbeat();
void MainStateMechine();
void simple_value();

double PID_plan2(double position_error, double velocity_error, uint8_t time_delay);
//void Joystick();

//I2C functions
void softResetMode();
void EmergencyMode();
void stopEmergencyMode();
void startTestMode();
void stopTestMode();
void startRunMode();
void stopRunMode();
void pickMode();
void placeMode();
void readStatus();
void modeControl();

//   1) soft Reset Mode
//   2) Emergency Mode
//   3) stop Emergency Mode
//   4) start Test Mode
//   5) stop Test Mode
//   6) start Run Mode
//   7) stop Run Mode
//   8) pick Mode
//   9) place Mode
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //end effector delay for SDA line error
  HAL_Delay(10);
  hmodbus.huart = &huart2;
  hmodbus.htim = &htim11;
  hmodbus.slaveAddress = 0x15;
  hmodbus.RegisterSize =70;
  Modbus_init(&hmodbus, registerFrame);

  //Joystick setup
  HAL_ADC_Start_DMA(&hadc1, &adc, 3);

  //encoder setup
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);


  // motor setup
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);

  // soft reset I2C evrytime we start new run
  modeSelect = 1;
  static uint32_t main_timestamp = 0;
  static uint32_t I2C_timestamp = 0;

  static uint8_t toggle_laser = 1;
  static uint8_t last_toggle_laser;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // for I2C delay call read and control
	  static uint32_t i2ctimestamp = 0;
	  static uint32_t i2ctimestamp2 = 0;
	  static uint32_t endstatustimestamp = 0;


	  debug_1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	  debug_2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);

  	  if(modeSelect != 0){
  		  modeControl();
  	  }

  	  if(readFlag == 1 && i2ctimestamp <= HAL_GetTick() )
  	  {
  	      i2ctimestamp = HAL_GetTick() + 10;
  	      readStatus();

  	  }

  	  if(readFlag == 1 && endstatustimestamp <= HAL_GetTick() ){
  		  endstatustimestamp += 10;
  	      ENEstatus = statusTemp[0];
  	  }

	  if(readFlag2 == 1 && i2ctimestamp2 <= HAL_GetTick() ){

		  i2ctimestamp2 = HAL_GetTick() + 1000;
		  readStatus();
		  read_count += 1;
		  if(read_count  >3){
			readFlag2 = 0;
		  }

	  }

	  if(endstatustimestamp <= HAL_GetTick() ){
		  endstatustimestamp += 10;
		  ENEstatus = statusTemp[0];
	  }


	 // I2C protocol frame
	 toggle_laser = registerFrame[2].U16;

	 if(toggle_laser == 1 && last_toggle_laser != toggle_laser){
		 modeSelect = 4;

	 }else if(toggle_laser == 0 && last_toggle_laser != toggle_laser){
		 modeSelect = 5;

	 }
	 last_toggle_laser = toggle_laser;



	  simple_value();

	  // Modbus Protocal

	  Modbus_Protocal_Worker();

	  // main state machine and heartbeat

	  MainStateMechine();
	  heartbeat();


//	  if(hi2c1.State != HAL_I2C_STATE_BUSY){
//		  status = readStatus();
//	  }

	  registerFrame[17].U16 = position;

	  // make position, velocuty, accerelation




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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 20000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 2005;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim11, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 1433;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void heartbeat()
{
	static uint32_t timestamp = 0;
	if(HAL_GetTick() >= timestamp)
	{
		timestamp = HAL_GetTick() + 200;
		registerFrame[0].U16 = 22881;
	}
}

void simple_value(){
	static uint32_t timestamp = 0;

	//encoder value
	QEIRead = __HAL_TIM_GET_COUNTER(&htim2);

	//position
	position = QEIRead / 128.38046;

	registerFrame[17].U16 = position * 10;

	registerFrame[18].U16 = velocity * 10;

	registerFrame[19].U16 = accerelation * 10;
	//velocity and accerelation

	if(timestamp < HAL_GetTick()){
		timestamp = HAL_GetTick() + 1;

		static float previousPosition = 0;
		static float previousVelocity = 0;

		float deltaTime = 1.0 / 1000.0;

		velocity = (position - previousPosition) / deltaTime;

		velocity_array[velocity_index] = velocity;
		velocity_index++;
		velocity_index %= 10;

		float sum = 0.0;
		for(int i = 0; i < 10; i++){
			sum += velocity_array[i];
		}

		avg_velocity = sum / 10.0;

		accerelation = (velocity - previousVelocity) / deltaTime;


		previousPosition = position;

		previousVelocity = velocity;

	}


}

void Joystick()
{
	// X_Axis
	static uint8_t last_toggle;
	debug_3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);

	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 0 && last_toggle == 1){
		static uint8_t i = 0;
		static uint8_t j = 0;
		if(pick_or_place == 0){


			pick_point.X_point[i] = registerFrame[68].U16 / 10.0; // get the position from
			pick_point.Y_point[i] = position;

			i++;
			i%=3;

		}else if(pick_or_place == 1){

			place_point.X_point[j] = registerFrame[68].U16 / 10.0;
			place_point.Y_point[j] = position;

			j++;
			j%=3;

		}
	}
	if (adc.X_Axis >= 0 && adc.X_Axis <= 100) {
	    right = 1;
	    left = 0;
	    registerFrame[64].U16 = 8;
	}

	else if(adc.X_Axis >= 3100)
	{
		left = 1;
		right = 0;
		registerFrame[64].U16 = 4;
	}else{
		left = 0;
		right = 0;
		registerFrame[64].U16 = 0;
	}

	//Y_Axis
	if(0 <= adc.Y_Axis && adc.Y_Axis <= 100)
	{
		top = 1;
		bottom = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 2500);
     //   HAL_TIM_
	}
	else if(adc.Y_Axis >= 3150)
	{
		bottom = 1;
		top = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 2500);
	}else{
		bottom = 0;
		top = 0;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	}
	last_toggle = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
}

void main_trajectory(float required_position, uint8_t point_or_tray){
	static uint32_t timestamp_trajectory = 0;
	if(timestamp_trajectory < HAL_GetTick()){
		timestamp_trajectory = HAL_GetTick() + 1;
		sampling_time = sampling_time + 1.0;
		if(point_or_tray == 1){
			Trapzoidal_viapoint(position, required_position);
		}else if(point_or_tray == 0){
			Trapzoidal_pick_place(position, required_position);
		}
	}


}


void Trapzoidal_viapoint(double initial_point, double final_point){
	// setup rise_time = 0.15s || stable = 0.70 || fall_time = 0.15
	float compare_time;
	double maximum_speed = 700; // mm/s
	float time_first;
	static double last_position; //mm^2/s


	compare_time = sampling_time / 1000.0;



	time_final = (1.5* (final_point - initial_point) / maximum_speed);
	time_first = (initial_point - final_point + maximum_speed*time_final)/maximum_speed;

	if(time_final >= 0){
		if(compare_time < time_first){
				present_position = initial_point + maximum_speed*(compare_time*compare_time)/(2*time_first);
			}
			else if(compare_time > time_first && compare_time < time_final - time_first){
				present_position = (final_point + initial_point - maximum_speed * time_final)/2 + (maximum_speed * compare_time);
			}
			else if(time_final - time_first < compare_time && compare_time <= time_final){
				present_position = final_point - (maximum_speed*(time_final*time_final)) / (2*time_first) + maximum_speed*time_final*compare_time/time_first - maximum_speed*(compare_time*compare_time)/(2*time_first);
			}
			else if(compare_time > time_final){
//				present_time = 0;
//				position = present_position;

			}

	}else if(time_final < 0){
		time_final = time_final * -1;
		time_first = time_first * -1;
		if(compare_time < time_first){
				present_position = initial_point - maximum_speed*(compare_time*compare_time)/(2*time_first);
		}
		else if(compare_time > time_first && compare_time < time_final - time_first){
				present_position = (final_point + initial_point + maximum_speed * time_final)/2 - (maximum_speed * compare_time);
		}
		else if(time_final - time_first < compare_time && compare_time <= time_final){
				present_position = final_point + (maximum_speed*(time_final*time_final)) / (2*time_first) - maximum_speed*time_final*compare_time/time_first + maximum_speed*(compare_time*compare_time)/(2*time_first);
		}
		else if(compare_time > time_final){
//				position = present_position;

		}
	}

	speed_happen = (present_position - last_position)/0.001;

	degree = 128.38046 * present_position;
	Vfeedback = PID_plan2(degree - QEIRead, speed_happen - avg_velocity, 1);
	if(Vfeedback >= 20000){
		Vfeedback = 20000;
	}

	else if(Vfeedback <= -20000){
		Vfeedback = -20000;
	}

	motor_control(compare_time, time_final);
	last_position = present_position;
}


void Trapzoidal_pick_place(double initial_point, double final_point){
	// setup rise_time = 0.15s || stable = 0.70 || fall_time = 0.15
	float compare_time;
	double maximum_speed = 700; // mm/s
	float time_first;
	static double last_position; //mm^2/s


	compare_time = sampling_time / 1000.0;



	time_final = (1.5* (final_point - initial_point) / maximum_speed);
	time_first = (initial_point - final_point + maximum_speed*time_final)/maximum_speed;

	if(time_final >= 0){
		if(compare_time < time_first){
				present_position = initial_point + maximum_speed*(compare_time*compare_time)/(2*time_first);
			}
			else if(compare_time > time_first && compare_time < time_final - time_first){
				present_position = (final_point + initial_point - maximum_speed * time_final)/2 + (maximum_speed * compare_time);
			}
			else if(time_final - time_first < compare_time && compare_time <= time_final){
				present_position = final_point - (maximum_speed*(time_final*time_final)) / (2*time_first) + maximum_speed*time_final*compare_time/time_first - maximum_speed*(compare_time*compare_time)/(2*time_first);
			}
			else if(compare_time > time_final){
//				present_time = 0;
//				position = present_position;

			}

	}else if(time_final < 0){
		time_final = time_final * -1;
		time_first = time_first * -1;
		if(compare_time < time_first){
				present_position = initial_point - maximum_speed*(compare_time*compare_time)/(2*time_first);
		}
		else if(compare_time > time_first && compare_time < time_final - time_first){
				present_position = (final_point + initial_point + maximum_speed * time_final)/2 - (maximum_speed * compare_time);
		}
		else if(time_final - time_first < compare_time && compare_time <= time_final){
				present_position = final_point + (maximum_speed*(time_final*time_final)) / (2*time_first) - maximum_speed*time_final*compare_time/time_first + maximum_speed*(compare_time*compare_time)/(2*time_first);
		}
		else if(compare_time > time_final){
//				position = present_position;

		}
	}

	speed_happen = (present_position - last_position)/0.001;

	degree = 128.38046 * present_position;
	Vfeedback = PID_plan2(degree - QEIRead, speed_happen - avg_velocity, 1);
	if(Vfeedback >= 20000){
		Vfeedback = 20000;
	}

	else if(Vfeedback <= -20000){
		Vfeedback = -20000;
	}

	motor_control_pick_place(compare_time, time_final);
	last_position = present_position;
}




double PID_plan2(double position_error, double velocity_error, uint8_t time_delay) {
    // Position PID Constants
    double Kp_position = 1.25;
    double Ki_position = 0.000030;

    // Velocity PID Constants
    double Kp_velocity = 0.85;
    double Ki_velocity = 0.000050;

    // Variables for position PID controller
    static double position_integral = 0;

    // Variables for velocity PID controller
    static double velocity_integral = 0;
    static double velocity_error_previous = 0;

    // Calculate position PID output
    double position_output = Kp_position * position_error + Ki_position * position_integral;

    // Update position integral
    position_integral += position_error * (time_delay / 1000.0);

    // Calculate velocity PID output
    double velocity_output = Kp_velocity * velocity_error + Ki_velocity * velocity_integral;

    // Update velocity integral
    velocity_integral += velocity_error * (time_delay / 1000.0);

    // Store current velocity error for the next iteration
    velocity_error_previous = velocity_error;

    // Calculate the final output as the sum of position and velocity outputs
    double final_output = position_output + velocity_output;

    return final_output;
}


void motor_control(float present_time, float time)
{
	static uint16_t timestamp_motor = 10000;
	error_hap = registerFrame[49].U16/10.0 - position;
//	float drive = VIn * 200; // 1000 * VIn/5    5 volts

	if(Vfeedback >= 20000){
		Vfeedback = 20000;
	}

	else if(Vfeedback <= -20000){
		Vfeedback = -20000;
	}

//	if(setposition - 200 < Vfeedback > setposition + 200){
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//	}
	if ((error_hap) >= -0.1 && (error_hap) <= 0.1) {
//		present_time = 0;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);

		if(timestamp_motor < HAL_GetTick()){
		   timestamp_motor = HAL_GetTick() + 10000;
		   toggle_trajec = 0;
		   mainstate = checkstate;
		}

//		in_or_not = 0;
//		point_arrived = 1;
	}
	else if(Vfeedback > 0){

		if(Vfeedback <= 1700 && sampling_time/1000.0 >= time_final * 0.85){
			debug_1 = 1;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1700); //1830

		}else{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Vfeedback);
		}

	}

	else if(Vfeedback < 0){
		if(-1*Vfeedback <= 1700 && sampling_time/1000.0 >= time_final * 0.85){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1700);
		}else{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, -1*Vfeedback);
		}

	}

	else if(Vfeedback == 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	}
}


void motor_control_pick_place(float present_time, float time)
{
	static uint16_t timestamp_motor = 10000;
	error_hap = registerFrame[49].U16/10.0 - position;
//	float drive = VIn * 200; // 1000 * VIn/5    5 volts

	if(Vfeedback >= 20000){
		Vfeedback = 20000;
	}

	else if(Vfeedback <= -20000){
		Vfeedback = -20000;
	}

//	if(setposition - 200 < Vfeedback > setposition + 200){
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//	}
	if ((error_hap) >= -0.1 && (error_hap) <= 0.1) {
//		present_time = 0;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);

		arrived = 1;
	}
	else if(Vfeedback > 0){

		if(Vfeedback <= 1700 && sampling_time/1000.0 >= time_final * 0.85 && position > 0){
			debug_1 = 1;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1700); //1830

		}else{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Vfeedback);
		}

	}

	else if(Vfeedback < 0){
		if(-1*Vfeedback <= 1700 && sampling_time/1000.0 >= time_final * 0.85 && position < 0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1700);
		}else{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, -1*Vfeedback);
		}

	}

	else if(Vfeedback == 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
	}
}
uint8_t find_diagonal_X(){

	if(!(pick_point.X_point[0] - pick_point.X_point[1] >= -5 && pick_point.X_point[0] - pick_point.X_point[1] <= 5) && !(pick_point.Y_point[0] - pick_point.Y_point[1] >= -5 && pick_point.Y_point[0] - pick_point.Y_point[1] <= 5)){
		return 1;
	}
	else if(!(pick_point.X_point[1] - pick_point.X_point[2] >= -5 && pick_point.X_point[1] - pick_point.X_point[2] <= 5) && !(pick_point.Y_point[1] - pick_point.Y_point[2] >= -5 && pick_point.Y_point[1] - pick_point.Y_point[2] <= 5)){
		return 3;
	}
	else if(!(pick_point.X_point[0] - pick_point.X_point[2] >= -5 && pick_point.X_point[0] - pick_point.X_point[2] <= 5) && !(pick_point.Y_point[0] - pick_point.Y_point[2] >= -5 && pick_point.Y_point[0] - pick_point.Y_point[2] <= 5)){
		return 2;
	}else{
		return 0;
	}
}

uint8_t find_diagonal_Y(){

	if(!(place_point.X_point[0] - place_point.X_point[1] >= -5 && place_point.X_point[0] - place_point.X_point[1] <= 5) && !(place_point.Y_point[0] - place_point.Y_point[1] >= -5 && place_point.Y_point[0] - place_point.Y_point[1] <= 5)){
		return 1;

	}
	else if(!(place_point.X_point[1] - place_point.X_point[2] >= -5 && place_point.X_point[1] - place_point.X_point[2] <= 5) && !(place_point.Y_point[1] - place_point.Y_point[2] >= -5 && place_point.Y_point[1] - place_point.Y_point[2] <= 5)){
		return 3;

	}
	else if(!(place_point.X_point[0] - place_point.X_point[2] >= -5 && place_point.X_point[0] - place_point.X_point[2] <= 5) && !(place_point.Y_point[0] - place_point.Y_point[2] >= -5 && place_point.Y_point[0] - place_point.Y_point[2] <= 5)){
		return 2;

	}else{
		return 0;
	}


}

void calculate(){
	static float height,width;
	if(find_diagonal_X() == 1){

		height = pick_point.X_point[0] - pick_point.X_point[1];
		width = pick_point.Y_point[0] - pick_point.Y_point[1];

		if(height > width){
			degree_triangle = atan(height/width);
		}else if(width < height){
			degree_triangle = atan(width/height);
		}
	}else if(find_diagonal_X() == 2){

		height = pick_point.X_point[1] - pick_point.X_point[2];
		width = pick_point.Y_point[1] - pick_point.Y_point[2];

		if(height > width){
			degree_triangle = atan(height/width);
		}else if(width < height){
			degree_triangle = atan(width/height);
		}

	}else if(find_diagonal_X() == 3){

		height = pick_point.X_point[0] - pick_point.X_point[2];
		width = pick_point.Y_point[0] - pick_point.Y_point[2];

		if(height > width){
			degree_triangle = atan(height/width);
		}else if(width < height){
			degree_triangle = atan(width/height);
		}

	}



}



void home_setpoint(){
	debug_1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	debug_2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)){
			flip_position = 1;
		}
		if(flip_position){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 2000);
		}else if(!flip_position){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 2000);
		}

	}else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		flip_position = 0;
		registerFrame[16].U16 = 0;
		registerFrame[1].U16 = 0;
		mainstate = checkstate;
	}
}


void MainStateMechine()
{
	static uint16_t stack = 0;
	static uint8_t hold_status = 0;
	static uint32_t timestamp = 0;
	static uint8_t in_out = 0;
	static uint8_t array_place = 0;

		if(HAL_GetTick() >= timestamp)
		{
			timestamp = HAL_GetTick() + 1;
			switch (mainstate)
			{
				case checkstate: //base system status
					if(registerFrame[1].U16 == 1) // set pick tray
					{
						mainstate = SetPick_PlaceTray;
						registerFrame[16].U16 = 0;
						pick_or_place = 0;
					}
					else if(registerFrame[1].U16 == 2) // set place tray
					{
						mainstate = SetPick_PlaceTray;
						registerFrame[16].U16 = 1;
						pick_or_place = 1;
					}
					else if(registerFrame[1].U16 == 4) // set home
					{
						mainstate = Homing;
						home_in = 1;
						registerFrame[16].U16 = 2;
					}
					else if(registerFrame[1].U16 == 8) //  run tray mode
					{
						mainstate = Run_tray_mode;
					}
					else if(registerFrame[1].U16 == 16) // run point mode
					{
						mainstate = SetPoint;

					}

				break;

				case SetPoint:

					if(toggle_trajec){
						registerFrame[65].U16 = registerFrame[48].U16;//x axis target pos. = goal point x
						registerFrame[66].U16 = 300*10;//max speed x axis
						registerFrame[67].U16 = 1;//max acceleration x axis
						registerFrame[64].U16 = 2;//run x axis
					}else if(!toggle_trajec){
						registerFrame[64].U16 = 0;
						registerFrame[65].U16 = 0;
						registerFrame[66].U16 = 0;
						registerFrame[67].U16 = 0;//max acceleration x axis
					}

					if(registerFrame[49].U16 != payback_val){
						sampling_time = 0;
						payback_val = registerFrame[49].U16;
					}

					main_trajectory(registerFrame[49].U16/10.0, 1);




//					registerFrame[1].U16 = 0;

					mainstate = checkstate;


//					registerFrame[1].U16 = 0;

				break;

				case Homing:

					//reset base status
					if(home_in){
						registerFrame[16].U16 = 4;
						registerFrame[64].U16 = 1;//Home x-Axis
						home_in = 0;
					}

					home_setpoint();
					mainstate = checkstate;

				break;

				case SetPick_PlaceTray:

//					registerFrame[1].U16 = 0;//reset base status

					registerFrame[65].U16 = registerFrame[48].U16;//x axis target pos. = goal point x
					registerFrame[66].U16 = 50*10;//max speed x axis
					registerFrame[67].U16 = 3;//min acceleration x axis
					Joystick();


					mainstate = checkstate;

					if(registerFrame[64].U16 != 4 && registerFrame[64].U16 != 8)

						mainstate = checkstate;


				break;

				case Run_tray_mode:
					if(arrived){
						arrived = 0;
						main_trajectory(pick_place_array[array_place],0);
						array_place++;

						if(array_place == 10){
							mainstate = checkstate;
						}
					}


					//pick_place_array
					calculate();
					registerFrame[65].U16 = registerFrame[48].U16;


//					registerFrame[1].U16 = 0;

				break;
			}
			}

}


// I2C functions
//
// Soft Reset mode********************************
void softResetMode(){
	HAL_I2C_Master_Transmit_IT(&hi2c1, slaveAddress << 1, softResetseq, 4);
}
//*************************************************


//Emergency mode***********************************************
void EmergencyMode(){
     HAL_I2C_Master_Transmit_IT(&hi2c1, slaveAddress << 1, Emergencyseq, 1);
}

void stopEmergencyMode(){
	 HAL_I2C_Master_Transmit_IT(&hi2c1, slaveAddress << 1, stopEmergencyseq, 4);
}
//*************************************************************




//Test mode****************************************************
void startTestMode()
{
	HAL_I2C_Master_Transmit_IT(&hi2c1, slaveAddress << 1, startTestseq, 2);
}
void stopTestMode()
{
	HAL_I2C_Master_Transmit_IT(&hi2c1, slaveAddress << 1, stopTestseq, 2);
}
//**************************************************************



// ReadMode***************************************************

void readStatus(){
	HAL_I2C_Master_Receive_IT(&hi2c1, slaveAddress << 1, statusTemp, 1);
}
//*************************************************************



//Gripper**********************************************************

void startRunMode(void)
{
  HAL_I2C_Master_Transmit_IT(&hi2c1, slaveAddress << 1, startRunseq, 2);
}


void stopRunMode(void)
{
  HAL_I2C_Master_Transmit_IT(&hi2c1, slaveAddress << 1, stopRunseq, 2);
}


void pickMode(void)
{
  HAL_I2C_Master_Transmit_IT(&hi2c1, slaveAddress << 1, pickseq, 2);
}


void placeMode(void)
{
  HAL_I2C_Master_Transmit_IT(&hi2c1, slaveAddress << 1, placeseq, 2);
}
//************************************************************

void modeControl(){
	if(modeSelect == 0){
		readFlag2 = 0;

	}

	if( modeSelect == 1){
		softResetMode();
		modeSelect = 0;

	}
	if( modeSelect == 2){
		EmergencyMode();
		modeSelect = 0;

	}
	if( modeSelect == 3){
		stopEmergencyMode();
		modeSelect = 0;

	}
	if( modeSelect == 4){
		startTestMode();
		modeSelect = 0;

	}
	if( modeSelect == 5){
		stopTestMode();
		modeSelect = 0;

	}
	if( modeSelect == 6){
		startRunMode();
		modeSelect = 0;

	}
	if( modeSelect == 7){
		stopRunMode();
		modeSelect = 0;

	}
	if( modeSelect == 8){
		read_count = 0;
		pickMode();
          readFlag2 = 1;

		modeSelect = 0;

	}
	if( modeSelect == 9){
		read_count = 0;
		placeMode();
          readFlag2 = 1;

		modeSelect = 0;

	}
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
 readFlag = 1;
}



void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
 readFlag = 0;
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
