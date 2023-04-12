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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
#define PWM 1
#define SCTL 2
#define TOTAL_IC 1
#define DEBUG 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//CAN TX:PD1 RX:PD0
//SPI1  miso:PA6 mosi:PA7 cs:PA4 clk:PA5
//SPI2 miso:PC2 mosi:PB15 cs: clk:PB13

//fun notes from meeting
/*ADD HARDWARE NSS TO SPI FOR CHIP SELECT

	motor controller what to send
  		send can msg of current supply capabilties?
		gpio go or no go?
	what pieces of data does lv need from us - what data do we get from cells and how we communicate
	bms need low id number for high priority info such as availble current
	 */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
	// 25AA040A instructions
	/*const uint8_t EEPROM_READ = 0b00000011;
	const uint8_t EEPROM_WRITE = 0b00000010;
	const uint8_t EEPROM_WRDI = 0b00000100;
	const uint8_t EEPROM_WREN = 0b00000110;
	const uint8_t EEPROM_RDSR = 0b00000101;
	const uint8_t EEPROM_WRSR = 0b00000001;*/
	//int16_t pec15Table[256];
	int16_t CRC15_POLY = 0x4599;
	  uint8_t streg=0;
	  int8_t error = 0;
	  uint32_t conv_time = 0;
	  int8_t s_pin_read=0;
	/********************************************************************
	 ADC Command Configurations. See LTC681x.h for options(Reference Linduino LTC6813)
	*********************************************************************/
	const uint8_t ADC_OPT = ADC_OPT_DISABLED; //!< ADC Mode option bit
	const uint8_t ADC_CONVERSION_MODE =MD_7KHZ_3KHZ; //!< ADC Mode
	const uint8_t ADC_DCP = DCP_DISABLED; //!< Discharge Permitted
	const uint8_t CELL_CH_TO_CONVERT =CELL_CH_ALL; //!< Channel Selection for ADC conversion
	const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; //!< Channel Selection for ADC conversion
	const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; //!< Channel Selection for ADC conversion
	const uint8_t SEL_ALL_REG = REG_ALL; //!< Register Selection
	const uint8_t SEL_REG_A = REG_1; //!< Register Selection
	const uint8_t SEL_REG_B = REG_2; //!< Register Selection

	const uint16_t MEASUREMENT_LOOP_TIME = 500; //!< Loop Time in milliseconds(ms)

	//Under Voltage and Over Voltage Thresholds
	const uint16_t OV_THRESHOLD = 41000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
	const uint16_t UV_THRESHOLD = 30000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

	//Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL CAPS
	const uint8_t WRITE_CONFIG = DISABLED;  //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
	const uint8_t READ_CONFIG = DISABLED; //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
	const uint8_t MEASURE_CELL = ENABLED; //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
	const uint8_t MEASURE_AUX = DISABLED; //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
	const uint8_t MEASURE_STAT = DISABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
	const uint8_t PRINT_PEC = DISABLED; //!< This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop
	/************************************
	  END SETUP
	*************************************/
	/*******************************************************
	 Global Battery Variables received from 681x commands
	 These variables store the results from the LTC6813
	 register reads and the array lengths must be based
	 on the number of ICs on the stack
	 ******************************************************/
	cell_asic BMS_IC[TOTAL_IC]; //!< Global Battery Variable

	/*************************************************************************
	 Set configuration register. Refer to the data sheet
	**************************************************************************/
	uint8_t REFON = 1; //!< Reference Powered Up Bit
	uint8_t ADCOPT = 0; //!< ADC Mode option bit
	uint8_t GPIOBITS_A[5] = {0,0,1,1,1}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
	uint8_t GPIOBITS_B[4] = {0,0,0,0}; //!< GPIO Pin Control // Gpio 6,7,8,9
	uint16_t UV=UV_THRESHOLD; //!< Under voltage Comparison Voltage
	uint16_t OV=OV_THRESHOLD; //!< Over voltage Comparison Voltage
	uint8_t DCCBITS_A[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
	uint8_t DCCBITS_B[7]= {0,0,0,0,0,0,0}; //!< Discharge cell switch //Dcc 0,13,14,15
	uint8_t DCTOBITS[4] = {1,0,1,0}; //!< Discharge time value //Dcto 0,1,2,3  // Programed for 4 min
	/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */
	uint8_t FDRF = 0; //!< Force Digital Redundancy Failure Bit
	uint8_t DTMEN = 1; //!< Enable Discharge Timer Monitor
	uint8_t PSBITS[2]= {0,0}; //!< Digital Redundancy Path Selection//ps-0,1

	uint8_t UART2_rxBuffer[100] = {0};

	uint8_t stop_flag = 0;
	uint8_t uart_cmd = 0;

	// Menu string to display menu to user
	char menu[500] = {"\
	\r\n============ stm32-cli =============\
	\r\nPrint Menu		         ----> m\
	\r\nDebug	     			 ----> d\
	\r\nTest 1                   ----> 1\
	\r\nTest 2                   ----> 2\
	\r\nSPI_COMM		         ----> 3\
	\r\nTest 4	     			 ----> 4\
	\r\nTest 5	     			 ----> 5\
	\r\nOr, press button for accel. data.\
	\r\nType your option here: \r\n\r\n"};

	static app_data a_dd;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
uint32_t DWT_Delay_Init(void);
//void cmd_6813(uint8_t tx_cmd[2]);



#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
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
  app_data a_d;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  LTC6813_init_cfg(TOTAL_IC, BMS_IC);
  LTC6813_init_cfgb(TOTAL_IC,BMS_IC);
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++)
  {
    LTC6813_set_cfgr(current_ic,BMS_IC,REFON,ADCOPT,GPIOBITS_A,DCCBITS_A, DCTOBITS, UV, OV);
    LTC6813_set_cfgrb(current_ic,BMS_IC,FDRF,DTMEN,PSBITS,GPIOBITS_B,DCCBITS_B);
  }
  LTC6813_reset_crc_count(TOTAL_IC,BMS_IC);
  LTC6813_init_reg_limits(TOTAL_IC,BMS_IC);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USB_HOST_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();
  printf("broke\r\n");
  a_d.hcan1 = &hcan1;
  a_d.hspi1 = &hspi1;
  a_d.hspi2 = &hspi2;
  a_d.huart2 = &huart2;
  a_d.htim1 = &htim1;
  a_d.debug = DEBUG;

  HAL_UART_Receive_IT (&huart2, UART2_rxBuffer, 1);
  //while(1){}//remove once done testing interupts
  init_appdata(&a_d);
  printf("\r\nStarting Code\r\n");

  // CS pin should default high
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(1000);
  //turn off rgb leds
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //printf("entered the while\r\n");
	  if(UART2_rxBuffer[0] !=0){
		  switch(UART2_rxBuffer[0]){
		     if(stop_flag==1){
		     	stop_flag=0;
		     }
		     else{
		     	stop_flag=1;
		     }
		     //stop_flag=1;//check to see if interrupt will start running multiple tests at the same time if not running
		 		case '1':
		 			//run test 1
		 			test1();
		 			break;
		 		case '2':
		 			test2();
		 			break;
		 		case '3':
		 			test3();
		 			break;
		 		case '4':
		 			test4();
		 			break;
		 		case 'm':
		 			printf("%s",menu);
		 			stop_flag=0;
		 			break;
		 		default:
		 			printf("we dont like that command\r\n");
		     }
		     printf("stop flag: %d\r\n",stop_flag);
		     printf("UART2_rxBuffer: %x\r\n",UART2_rxBuffer[0]);


		     UART2_rxBuffer[0]=0;
	  }
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  sConfig.Channel = ADC_CHANNEL_11;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_CS_GPIO_Port, GPIO_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint32_t DWT_Delay_Init(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

    /* Check if clock cycle counter has started */
    if(DWT->CYCCNT)
    {
       return 0; /*clock cycle counter started*/
    }
    else
    {
      return 1; /*clock cycle counter not started*/
    }
}


PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int mode_flag = 0;//flag for knowing when to stay in a mode and when to exit safely
const int vbat_max = 4.2;//set to the max voltage cars cell can charge to immediately ceases charging once exceeded
const int vbat_mix = 3.2;//minimum voltage cars cells can be upon hitting immediately shutdown
const int current_max = 2;//max current allowed for charging system
int CCL = 2; // current charge limit - initially the max current a cell can handle
int DCL = 2; // discharge current limit - initially the max current a cell can handle
int hall_current = 0;
#define CHARGING 1
#define DISCHARGING 2
//assign AIR to GPIO PIN
void charging_mode(){//activated by GPIO Signal going high from external source(interrupt)
	while(mode_flag==CHARGING){
		//check if charge current limit >0
		if(CCL>0){
			//ADC of hall effect-at pin PC1
			if(hall_current>=CCL+2){//check if pack current >= charge current limit+xAmps(x=us defined threshold)
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//turn off charging

			}
			else{
				//allow charging to continue - repeats the loop
			}
		}
		else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//turn off power input
		}
	}
}

void discharge_mode(){//default mode ~when GPIO Signal is low
	while(mode_flag==DISCHARGING){
		//check if discharge current limit >0
		if(DCL>0){
			if(hall_current < DCL+2){//check if pack current >= discharge current limit+xAmps(x=us defined threshold)
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//turn off power output
			}
			else{

			}
		}
		else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//turn off power output
		}
	}
}

void print_conv_time(uint32_t conv_time)
{
  uint16_t m_factor=1000;  // to print in ms

  //Serial.print(F("Conversion completed in:"));
  //Serial.print(((float)conv_time/m_factor), 1);
  //Serial.println(F("ms \n"));
  printf("Conversion completed in %f ms\r\n",(float)conv_time/m_factor);
}

void check_error(int error)
{
  if (error == -1)
  {
    printf("A PEC error was detected in the received data\r\n");
  }
}

void print_cells(uint8_t datalog_en)
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      //Serial.print(" IC ");
      //Serial.print(current_ic+1,DEC);
      //Serial.print(", ");
      printf(" IC %d,",current_ic+1);
      for (int i=0; i<BMS_IC[0].ic_reg.cell_channels; i++)
      {
        //Serial.print(" C");
        //Serial.print(i+1,DEC);
        //Serial.print(":");
        //Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001,4);
        //Serial.print(",");
        printf(" C %d:%f,",i+1,BMS_IC[current_ic].cells.c_codes[i]*0.0001);
      }
      //Serial.println();
      printf("\r\n");
    }
    else
    {
      //Serial.print(" Cells, ");
      printf(" Cells, ");
      for (int i=0; i<BMS_IC[0].ic_reg.cell_channels; i++)
      {
        //Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001,4);
        //Serial.print(",");
        printf("%f,",BMS_IC[current_ic].cells.c_codes[i]*0.0001);
      }
    }
  }
  //Serial.println("\n");
  printf("\r\n");
}

/*!****************************************************************************
  \brief prints data which is written on COMM register onto the serial port
  @return void
 *****************************************************************************/
void print_wrcomm(void)
{
 int comm_pec;

  //Serial.println(F("Written Data in COMM Register: "));
  printf("Written Data in COMM Register: \r\n");
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    //Serial.print(F(" IC- "));
    //Serial.print(current_ic+1,DEC);
    printf(" IC- %d",current_ic+1);

    for(int i = 0; i < 6; i++)
    {
      //Serial.print(F(", 0x"));
      //serial_print_hex(BMS_IC[current_ic].com.tx_data[i]);
      printf(", %x",BMS_IC[current_ic].com.tx_data[i]);
    }
    //Serial.print(F(", Calculated PEC: 0x"));
    printf(", Calculated PEC: ");
    comm_pec = pec15_calc(6,&BMS_IC[current_ic].com.tx_data[0]);
    //serial_print_hex((uint8_t)(comm_pec>>8));
    printf("%x",comm_pec>>8);
    //Serial.print(F(", 0x"));
    //serial_print_hex((uint8_t)(comm_pec));
    printf(", %x\r\n",comm_pec);
    //Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Prints received data from COMM register onto the serial port
  @return void
 *****************************************************************************/
void print_rxcomm(void)
{
   printf("Received Data in COMM register:");
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    //Serial.print(F(" IC- "));
    //Serial.print(current_ic+1,DEC);
    printf(" IC- %d",current_ic+1);

    for(int i = 0; i < 6; i++)
    {
      //Serial.print(F(", 0x"));
      //serial_print_hex(BMS_IC[current_ic].com.rx_data[i]);
      printf(", %x",BMS_IC[current_ic].com.rx_data[i]);
    }
    //Serial.print(F(", Received PEC: 0x"));
    //serial_print_hex(BMS_IC[current_ic].com.rx_data[6]);
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].com.rx_data[7]);
    //Serial.println("\n");
    printf(", Received PEC: %x, %x\r\n",BMS_IC[current_ic].com.rx_data[6],BMS_IC[current_ic].com.rx_data[7]);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Transmit(&huart2, UART2_rxBuffer, 1, 100);
    //beeg_UART2_rxBuffer[i] = UART2_rxBuffer[i];
    //HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 1);
	//uart_cmd=UART2_rxBuffer[0];
    HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 1);
}

void test1(void){
	printf("\r\n entering test1\r\n;");
	  char spi_tx_buffer[200];
	  char spi_rx_buffer[200];
	  uint16_t spi_transfer_size = 200;
	  while (0)//stop_flag)
	  {
		  for(int i = 0; i<200; i++){
			  spi_tx_buffer[i] = i;
		  }
	         //printf("Hello World\n\r");//wont see until uart to usb recieved
	   	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	   	  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) spi_tx_buffer,(uint8_t *) spi_rx_buffer,spi_transfer_size,100);
	   	//spi_write_read(spi_tx_buffer,200,spi_rx_buffer,200);
	   	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	   	  for(int i = 0; i<200; i++){
	   		  printf("%d ",spi_rx_buffer[i]);
	   	  }
	         //turn off rgb leds
	         //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

	         HAL_Delay(1000);
		}
	  printf("\r\n exiting test1\r\n;");
}

void test2(void){
	printf("\r\n entering test2\r\n;");
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  while(stop_flag){
			HAL_SPI_Transmit(a_dd.hspi1, 0x02,1,100);
			//for(uint8_t i = 0; i<tx_len+rx_len; i++){
			  //	printf("%x\r\n",rx_data[i]);
			//  }
			u_sleep(100);
	  }
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(1000);
  	printf("\r\n exiting test2\r\n;");

}

void test3(void){
	printf("\r\n entering test3\r\n;");
	  while(stop_flag){
	      for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++)
	      {
	        //Communication control bits and communication data bytes. Refer to the data sheet.
	        BMS_IC[current_ic].com.tx_data[0]= 0x81; // Icom CSBM Low(8) + data D0 (0x11)
	        BMS_IC[current_ic].com.tx_data[1]= 0x10; // Fcom CSBM Low(0)
	        BMS_IC[current_ic].com.tx_data[2]= 0xA2; // Icom CSBM Falling Edge (A) + data D1 (0x25)
	        BMS_IC[current_ic].com.tx_data[3]= 0x50; // Fcom CSBM Low(0)
	        BMS_IC[current_ic].com.tx_data[4]= 0xA1; // Icom CSBM Falling Edge (A) + data D2 (0x17)
	        BMS_IC[current_ic].com.tx_data[5]= 0x79; // Fcom CSBM High(9)
	      }
	      wakeup_sleep(TOTAL_IC);
	      LTC6813_wrcomm(TOTAL_IC,BMS_IC);
	      print_wrcomm();

	      wakeup_idle(TOTAL_IC);
	      LTC6813_stcomm(3);

	      wakeup_idle(TOTAL_IC);
	      error = LTC6813_rdcomm(TOTAL_IC,BMS_IC);
	      check_error(error);
	      print_rxcomm();
	  }
	  printf("\r\n exiting test3\r\n;");
}

void test4(void){
	printf("\r\n entering test4\r\n;");
	  while (stop_flag)
	  {
		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		  //HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) spi_tx_buffer,(uint8_t *) spi_rx_buffer,spi_transfer_size,100);
		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		  printf("starting cell voltage reading loop\r\n");
		  //wakeup_sleep(TOTAL_IC);
		  //printf("pass1\r\n");
		  //LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
		  //printf("pass2\r\n");
		  //conv_time = LTC6813_pollAdc();
		  //printf("start ADC waiting 1 seconds\r\n");
		  //HAL_Delay(1000);

		  //printf("pass3\r\n");
		 // print_conv_time(conv_time);
		  //printf("Cell Voltages:\r\n");

	      wakeup_sleep(TOTAL_IC);
	      error = LTC6813_rdcv(SEL_ALL_REG,TOTAL_IC,BMS_IC); // Set to read back all cell voltage registers
	      check_error(error);
	      print_cells(DATALOG_DISABLED);

	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
	      HAL_Delay(1000);
	      //turn off rgb leds
	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
	      HAL_Delay(1000);
	  }
	  printf("\r\n exiting test4\r\n;");
}

void init_appdata(app_data *app_data_init)
{
	a_dd = *app_data_init;//placed in because pointers if not saved will not be able to be passed to other files

	if(a_dd.debug==1){
		printf("\r\nDebugging init_appdata(main)\r\n");
		int data[3],sent[3];
		sent[0]=0;
		while (sent[0]<3){
			printf("prescaler: %u\r\n",app_data_init->hspi1->Init.BaudRatePrescaler);
			printf("prescaler: %u\r\n",a_dd.hspi1->Init.BaudRatePrescaler);
			sent[0] +=1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(app_data_init->hspi1, (uint8_t *) sent,(uint8_t *) data,1,100);
			printf("data sent %d :: data in init: %d \r\n",sent[0],data[0]);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_Delay(1000);
		}
	}
	init_app_data_6813(&a_dd);
	init_app_data_help(&a_dd);
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
