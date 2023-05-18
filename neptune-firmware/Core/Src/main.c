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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "BMX055.h"
#include "Sensors.h"
#include "utils.h"
#include "packets.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float32_t hard_iron_offset_data[3] = { 46.5, 74.0, -44.0 };
float32_t soft_iron_offset_data[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

/* Definitions for SensorRead */
osThreadId_t SensorReadHandle;
const osThreadAttr_t SensorRead_attributes = {
  .name = "SensorRead",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for LoRa */
osThreadId_t LoRaHandle;
const osThreadAttr_t LoRa_attributes = {
  .name = "LoRa",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for ServoActuate */
osThreadId_t ServoActuateHandle;
const osThreadAttr_t ServoActuate_attributes = {
  .name = "ServoActuate",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for KalmanFilter */
osThreadId_t KalmanFilterHandle;
const osThreadAttr_t KalmanFilter_attributes = {
  .name = "KalmanFilter",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE BEGIN PV */
int SERVO_ENABLED = 0;  // disabled by default
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void start_sensor_reading(void *argument);
void start_LoRa_task(void *argument);
void start_servo_control(void *argument);
void start_kalman_filter(void *argument);

/* USER CODE BEGIN PFP */
BMX055_Handle bmx055;
BMX055_Data_Handle bmx055_data;
Motors motors;
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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* LoRa configurations */
	LoRa_Handle = newLoRa();
	LoRa_Handle.hSPIx = &hspi2;
	LoRa_Handle.CS_port = RF_CE_GPIO_Port;
	LoRa_Handle.CS_pin = RF_CE_Pin;
	LoRa_Handle.reset_port = RF_RESET_GPIO_Port;
	LoRa_Handle.reset_pin = RF_RESET_Pin;
	LoRa_Handle.DIO0_port = RF_10O_GPIO_Port;
	LoRa_Handle.DIO0_pin = RF_10O_Pin;

	LoRa_Handle.frequency = 915;
	LoRa_Handle.spredingFactor = SF_7;						// default = SF_7
	LoRa_Handle.bandWidth = BW_125KHz;				  	// default = BW_125KHz
	LoRa_Handle.crcRate = CR_4_5;						// default = CR_4_5
	LoRa_Handle.power = POWER_20db;					// default = 20db
	LoRa_Handle.overCurrentProtection = 120; 				// default = 100 mA
	LoRa_Handle.preamble = 8;		  					// default = 8;

	HAL_GPIO_WritePin(RF_CE_GPIO_Port, RF_CE_Pin, GPIO_PIN_SET);

  /* Assign magnetometer calibration constants */
	arm_mat_init_f32(&bmx055.mag_hard_iron_offsets, 3, 1, hard_iron_offset_data);
	arm_mat_init_f32(&bmx055.mag_soft_iron_offsets, 3, 3, soft_iron_offset_data);

  /* BMX055 configurations */
	bmx055.hspi = &hspi1;
	// Accelerometer parameters
	bmx055.acc_CS_port = ACC_CE_GPIO_Port;
	bmx055.acc_CS_pin = ACC_CE_Pin;
	bmx055.acc_range = BMX055_ACC_RANGE_4;
	bmx055.acc_bandwidth = BMX055_ACC_PMU_BW_7_81;

	// Gyroscope parameters
	bmx055.gyro_CS_port = GYR_CE_GPIO_Port;
	bmx055.gyro_CS_pin = GYR_CE_Pin;
	bmx055.gyro_range = BMX055_GYRO_RANGE_65_6;		// 500 deg/s
	bmx055.gyro_bandwidth = BMX055_GYRO_BW_64;

	// Magnetometer parameters
	bmx055.mag_CS_port = MAG_CE_GPIO_Port;
	bmx055.mag_CS_pin = MAG_CE_Pin;
	bmx055.mag_data_rate = BMX055_MAG_DATA_RATE_30;

  /* Servo configurations */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
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
/**
* @}
*/
/**
* @}
*/

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


//  	float accel_data[3];
//	  BMX055_readAccel(&bmx055, accel_data);
//
//    float roll;
//    float pitch;
//
//    get_roll_and_pitch(accel_data, &roll, &pitch);
//
//    char str_buff[50];
//    sprintf(str_buff, "Roll: %.2f\tPitch: %.2f\r\n", roll, pitch);
//   	CDC_Transmit_FS(str_buff, strlen(str_buff));
//


	// 5% duty cycle minimum, 10% maximum
	// 16 bit, 65536 increments

	// arr = 200, duty cycle = ccr / arr * 100
	// if (duty_cycle > 11) {
	// 	reverse = 1;
	// 	duty_cycle = 11-3.5;
	// } else if (duty_cycle < 4) {
	// 	reverse = 0;
	// 	duty_cycle = 4+3.5;
	// }

//	 sprintf(str_buff, "Cycle: %.1f%%\r\n", duty_cycle);
//	 CDC_Transmit_FS(str_buff, strlen(str_buff));

	// htim3.Instance->CCR1 = duty_cycle * 200 / 100;
	// htim3.Instance->CCR2 = duty_cycle * 200 / 100;

	// if (reverse) {
	// 	duty_cycle -= 3.5;
	// } else {
	// 	duty_cycle += 3.5;
	// }




	HAL_Delay(50);
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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  sConfig.Channel = ADC_CHANNEL_2;
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim3.Init.Prescaler = 9600-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 200-1;
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
  HAL_GPIO_WritePin(GPIOC, RF_RESET_Pin|RF_CE_Pin|SD_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ACC_CE_Pin|GYR_CE_Pin|MAG_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RF_10O_Pin */
  GPIO_InitStruct.Pin = RF_10O_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_10O_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_RESET_Pin RF_CE_Pin SD_CE_Pin */
  GPIO_InitStruct.Pin = RF_RESET_Pin|RF_CE_Pin|SD_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ACC_CE_Pin GYR_CE_Pin MAG_CE_Pin */
  GPIO_InitStruct.Pin = ACC_CE_Pin|GYR_CE_Pin|MAG_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_start_sensor_reading */
/**
  * @brief  Function implementing the SensorRead thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_sensor_reading */
void start_sensor_reading(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	// Init BMX055
	if (!BMX055_init(&bmx055)) {
		printf("[main] BMX055 failed to start\r\n");
	}
	BMX055_setInterrupts(&bmx055);

	uint8_t data;

	/* GPS config */
//  gps_init_tpv(&tpv);
//	GNSS_Init(&GNSS_Handle, GPS_UART);
//	GNSS_LoadConfig(&GNSS_Handle);
	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

//	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Sensor type that is ready when task is released
	uint32_t sensor_type;

	/* Infinite loop */
	for (;;) {
		// Wait for sensors to be ready before running task
		xTaskNotifyWait(0, 0, &sensor_type, (TickType_t) portMAX_DELAY);

		/* Check each sensor each loop for new data */
		switch (sensor_type) {
		case Accel_Sensor:
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle, Accel_Sensor);
			float accel_data[3];
			BMX055_readAccel(&bmx055, accel_data);
			BMX055_exp_filter(bmx055_data.accel, accel_data, bmx055_data.accel, sizeof(accel_data) / sizeof(int),
			ACCEL_ALPHA);
			break;

		case Gyro_Sensor:
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle, Gyro_Sensor);
			float gyro_data[3];
			BMX055_readGyro(&bmx055, gyro_data);
			BMX055_exp_filter(bmx055_data.gyro, gyro_data, bmx055_data.gyro, sizeof(gyro_data) / sizeof(int),
			GYRO_ALPHA);
			break;

		case Mag_Sensor:
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle, Mag_Sensor);
			BMX055_readCompensatedMag(&bmx055, bmx055_data.mag);
			break;

		case Accel_Sensor | Gyro_Sensor:
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle,
			Accel_Sensor | Gyro_Sensor);
			BMX055_readAccel(&bmx055, accel_data);
			BMX055_exp_filter(bmx055_data.accel, accel_data, bmx055_data.accel, sizeof(accel_data) / sizeof(int),
			ACCEL_ALPHA);
			BMX055_readGyro(&bmx055, gyro_data);
			BMX055_exp_filter(bmx055_data.gyro, gyro_data, bmx055_data.gyro, sizeof(gyro_data) / sizeof(int),
			GYRO_ALPHA);

			break;

		case Accel_Sensor | Mag_Sensor:
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle,
			Accel_Sensor | Mag_Sensor);
			BMX055_readAccel(&bmx055, accel_data);
			BMX055_exp_filter(bmx055_data.accel, accel_data, bmx055_data.accel, sizeof(accel_data) / sizeof(int),
			ACCEL_ALPHA);
			BMX055_readCompensatedMag(&bmx055, bmx055_data.mag);
			break;

		case Gyro_Sensor | Mag_Sensor:
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle,
			Gyro_Sensor | Mag_Sensor);
			BMX055_readGyro(&bmx055, gyro_data);
			BMX055_exp_filter(bmx055_data.gyro, gyro_data, bmx055_data.gyro, sizeof(gyro_data) / sizeof(int),
			GYRO_ALPHA);
			BMX055_readCompensatedMag(&bmx055, bmx055_data.mag);
			break;

		case Accel_Sensor | Gyro_Sensor | Mag_Sensor:
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle,
			Accel_Sensor | Gyro_Sensor | Mag_Sensor);
			BMX055_readAccel(&bmx055, accel_data);
			BMX055_exp_filter(bmx055_data.accel, accel_data, bmx055_data.accel, sizeof(accel_data) / sizeof(int),
			ACCEL_ALPHA);
			BMX055_readGyro(&bmx055, gyro_data);
			BMX055_exp_filter(bmx055_data.gyro, gyro_data, bmx055_data.gyro, sizeof(gyro_data) / sizeof(int),
			GYRO_ALPHA);
			BMX055_readCompensatedMag(&bmx055, bmx055_data.mag);
			break;

		default:
			break;
		}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_LoRa_task */
/**
* @brief Function implementing the LoRa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_LoRa_task */
void start_LoRa_task(void *argument)
{
    /* USER CODE BEGIN start_LoRa_task */
    LoRa_reset(&LoRa_Handle);
	LoRa_setModulation(&LoRa_Handle, LORA_MODULATION);
	if (LoRa_init(&LoRa_Handle) != LORA_OK) {
      CDC_Transmit_FS("LoRa connection failed\r\n", strlen("LoRa connection failed\r\n"));
	}

	LoRa_startReceiving(&LoRa_Handle);
  /* Infinite loop */
  for(;;)
  {
    // Wait for LoRa to be ready before running task
    xTaskNotifyWait(0, 0, NULL, (TickType_t) portMAX_DELAY);

    // Read bytes in FIFO buffer
    uint8_t read_data[255];
    size_t bytes_read = LoRa_receive(&LoRa_Handle, read_data, sizeof(read_data));

    switch (read_data[0]) {
      case SET_ANGLES:
        float x = read_data[1] << 24;
        x = ((unsigned long) x) | (read_data[2] << 16);
        x = ((unsigned long) x) | (read_data[3] << 8);
        x = ((unsigned long) x) | read_data[4];

        float y = read_data[5] << 24;
        y = ((unsigned long) y) | (read_data[6] << 16);
        y = ((unsigned long) y) | (read_data[7] << 8);
        y = ((unsigned long) y) | read_data[8];

        set_motor(1, x, htim3);
        set_motor(2, y, htim3);
        break;
      case GIMBLE_MOTORS:
        gimble_test(htim3);
        break;
      case PONG:
        uint8_t resp = 1;
        LoRa_transmit(&LoRa_Handle, &resp, 1, portMAX_DELAY);
      default:
        break;
    }
  }
  /* USER CODE END start_LoRa_task */
}

/* USER CODE BEGIN Header_start_servo_control */
/**
* @brief Function implementing the ServoActuate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_servo_control */
void start_servo_control(void *argument)
{
  /* USER CODE BEGIN start_servo_control */
  /* Infinite loop */
  for(;;)
  {
    if (SERVO_ENABLED) {
        set_motor(1, motors->m1_angle, htim3);
        set_motor(2, motors->m2_angle, htim3);
    }
  }
  /* USER CODE END start_servo_control */
}

/* USER CODE BEGIN Header_start_kalman_filter */
/**
* @brief Function implementing the KalmanFilter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_kalman_filter */
void start_kalman_filter(void *argument)
{
  /* USER CODE BEGIN start_kalman_filter */
	uint32_t currentSampleTime = 0;
	uint32_t lastSampleTime = 0;
	int correct_freq = 1;
	int idx = 0;
	EKF_Init(&ekf, qu, EKF_K, EKF_P, EKF_Q, EKF_R, 0.0);
	float fake_acc_dat[10][3] = { { 0, 0, -9.81 }, { 1.703, 0, 9.66 }, { 3.355, 0, 9.218 }, { 4.905, 0, 8.496 }, { 6.306, 0, 7.515 }, { 7.515, 0, 6.306 }, { 8.496, 0, 4.905 }, { 9.218, 0, 3.355 }, { 9.66, 0, 1.703 }, { -9.81, 0, 0 } };
	float fake_gyr_dat[10][3] = { { 0, 0.175, 0 }, { 0, 0.175, 0 }, { 0, 0.175, 0 }, { 0, 0.175, 0 }, { 0, 0.175, 0 }, { 0, 0.175, 0 }, { 0, 0.175, 0 }, { 0, 0.175, 0 }, { 0, 0.175, 0 }, { 0, 0.175, 0 } };
//	osDelay(3000);

	/* Infinite loop */
	for (;;) {
		trace_counter++;
		currentSampleTime = micros(Micros_Timer);
		float dt = (currentSampleTime - lastSampleTime) / 1E6;
		lastSampleTime = currentSampleTime;

		float p = (float) (bmx055_data.gyro[0]);
		float q = (float) (bmx055_data.gyro[1]);
		float r = (float) (bmx055_data.gyro[2]);
		EKF_Predict(&ekf, p, q, r, dt);
		if (idx % correct_freq == 0) {
			EKF_Update(&ekf, (float) bmx055_data.accel[0] / 100, (float) bmx055_data.accel[1] / 100, (float) bmx055_data.accel[2] / 100, 1.5, 0, 0);
			idx = 0;
		}
		idx++;
		float euler_result[3];
		EP2Euler321(ekf.qu_data, euler_result);
		char printData[256];
		size_t sz;
		// Print pqr rates in rad/s
//		sz = snprintf(printData, sizeof(printData), "%0.6f, %0.6f, %0.6f, %0.6f\r\n", p, q, r, dt);
//		debug_print(printData, sz);
		// Print Euler angles
//		sz = snprintf(printData, sizeof(printData), "roll:%0.2f pitch:%0.2f yaw:%0.2f\r\n",
//				euler_result[0] * 57.2958, euler_result[1] * 57.2958,
//				euler_result[2] * 57.2958);
//		debug_print(printData, sz);
		// Print quaternions
//	sz = snprintf(printData, sizeof(printData), "%0.10f, %0.10f, %0.10f, %0.10f, %0.10f, %0.10f, %0.10f, %0.10f, %0.10f, %0.10f, %0.10f\r\n", p, q, r, dt, ekf.qu_data[0],
//				ekf.qu_data[1], ekf.qu_data[2], ekf.qu_data[3], euler_result[0], euler_result[1], euler_result[2]);
//		debug_print(printData, sz);
//		sz = snprintf(printData, sizeof(printData), "Orientation: %0.4f, %0.4f, %0.4f\r\n", euler_result[0] * 57.2958, euler_result[1] * 57.2958, euler_result[2] * 57.2958);
//		debug_print(printData, sz, dbg=DBG);
		osDelay(10);
//		for (int idx = 0; idx < 10; idx++) {
//			EKF_Predict(&ekf, fake_gyr_dat[idx][0], fake_gyr_dat[idx][1], fake_gyr_dat[idx][2], 1.0);
//			EKF_Update(&ekf, fake_acc_dat[idx][0], fake_acc_dat[idx][1], fake_acc_dat[idx][2], 1.5, 0, 0);
//
//			float euler_result[3];
//			EP2Euler123(ekf.qu_data, euler_result);
//			char printData[128];
//			size_t sz = snprintf(printData, sizeof(printData), "%0.2f, %0.2f, %0.2f\r\n", euler_result[0] * 57.2958,
//					euler_result[1] * 57.2958, euler_result[2] * 57.2958);
//			debug_print(printData, sz);
//			osDelay(1000);
//		}
	}
  /* USER CODE END start_kalman_filter */
}

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
