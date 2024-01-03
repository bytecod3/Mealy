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
#include "string.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include <stdio.h>
#include <stdarg.h> // for va_list and va_arg functions

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	ENABLE_FILE_WRITE 	0  /* set to 1 to enable writing data to SD card*/
#define ENABLE_FILTER		0  /* set to 1 to enable complementary filter */

#define ONE_G 				(9.81)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
char msg[20] = "";
char mpu_data[100];
char data[40] = ""; // sd card test char array

// create accelerometer struct
MPU6050 acc;
MPU6050* p_acc = &acc; // pointer to accelerometer
float x_acc_ms, y_acc_ms, z_acc_ms;

// struct to hold both accelerometer data and the gyroscope data
typedef struct {
	float x_acc;
	float y_acc;
	float z_acc;
	float gyro_x;
	float gyro_y;
	float gyro_z;
} SensorData;

SensorData sensorData;

// use this pointer to store raw sensor data
SensorData* p_SensorData = &sensorData;

// struct to hold filtered sensor data - angles to be specific
// these are angle estimates, not accurate angles
typedef struct {
	float x_angle_estimate;
	float y_angle_estimate;
	float z_angle_estimate;

} FilteredAngles;
FilteredAngles filtered_angles;
FilteredAngles* p_filtered_angles = &filtered_angles;

// variables to store the rotation angles of the sensor
unsigned long 	last_read_time;
float			last_x_angle; // filtered angles
float 			last_y_angle;
float			last_z_angle;
float			last_gyro_x_angle; // store the gyro angles to compare drift
float			last_gyro_y_angle;
float 			last_gyro_z_angle;

// radians to degrees conversion factor
float RADS_TO_DEG = 180.0 / 3.14159;
float alpha = 0.96;

// function to set the current angle readings to the last read angles
void set_last_read_angle_data(
		unsigned long time,
		float x,
		float y,
		float z,
		float x_gyro,
		float y_gyro,
		float z_gyro
		) {
	last_read_time = time;
	last_x_angle = x;
	last_y_angle = y;
	last_z_angle = z;
	last_gyro_x_angle = x_gyro;
	last_gyro_y_angle = y_gyro;
	last_gyro_z_angle = z_gyro;
}

// calibration variables
float	base_x_accel;
float 	base_y_accel;
float	base_z_accel;
float 	base_x_gyro;
float	base_y_gyro;
float	base_z_gyro;

// function to callibrate sensors
void callibrate_sensor(){
	int		num_readings = 10;
	float 	x_accel = 0;
	float	y_accel = 0;
	float 	z_accel	= 0;
	float	x_gyro	= 0;
	float 	y_gyro	= 0;
	float	z_gyro	= 0;

	// read and average the raw values from the IMU
	myPrintf("Callibrating sensor...\r");

	for(int i = 0; i < num_readings; i++){
		// read accelerometer
		MPU6050_ReadAcceleration(&acc);

		// Read gyroscope values
		MPU6050_ReadGyroscope(&acc);

		// convert accelerometer readings to m/s^2
		x_acc_ms = p_acc->acc_mps2[0] * ONE_G;
		y_acc_ms = p_acc->acc_mps2[1] * ONE_G;
		z_acc_ms = p_acc->acc_mps2[2] * ONE_G;

		x_accel += x_acc_ms;
		y_accel += y_acc_ms;
		z_accel += z_acc_ms;
		x_gyro += p_acc->gyro_data[0];
		y_gyro += p_acc->gyro_data[1];
		z_gyro += p_acc->gyro_data[2];

		HAL_Delay(100);

	}

	// average the readings
	x_accel /= num_readings;
	y_accel /= num_readings;
	z_accel /= num_readings;
	x_gyro /= num_readings;
	y_gyro /= num_readings;
	z_gyro /= num_readings;

	// Store the raw calibration values globally
	base_x_accel = x_accel;
	base_y_accel = y_accel;
	base_z_accel = z_accel;
	base_x_gyro = x_gyro;
	base_y_gyro = y_gyro;
	base_z_gyro = z_gyro;

	myPrintf("Callibration done!\r");

}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI4_Init(void);

/* USER CODE BEGIN PFP */
void myPrintf(const char *fmt, ...);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myPrintf(const char* fmt, ...){
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, -1);
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_SPI4_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */

	#if ENABLE_FILE_WRITE /* enable file write operations */

		  /*-------------------------FILE HANDLING FUNCTIONS--------------------------------*/
		  // this code was used for testing out the filtering algorithms
		  // initialise the SD card
		  HAL_Delay(1000);


		  FATFS FatFs; // fatfs handle
		  FIL fil; // file handle
		  FRESULT fres; // result after operations
		  BYTE readBuf[50];

		  // Open the file system
		  fres = f_mount(&FatFs, "", 1); //1=mount now
		  if (fres != FR_OK) {
			  myPrintf("[-]f_mount error (%i) \n", fres);
			  while(1);
		  } else {
			  myPrintf("[+] SD Card Init OK! \r\n");
		  }

	#endif

  /*-------------------------INERTIAL MEASUREMENT UNIT FUNCTIONS--------------------------------*/
  MPU6050_Initialise(&acc, &hi2c1);// TODO: check init status here

  // calibrate the sensor
  callibrate_sensor();

  // initialise the sensor angles to known values
  set_last_read_angle_data(HAL_GetTick(), 0, 0, 0, 0, 0, 0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// read accelerometer values
	MPU6050_ReadAcceleration(&acc);

	// Read gyroscope values
	MPU6050_ReadGyroscope(&acc);

	// convert accelerometer readings to m/s^2
	x_acc_ms = p_acc->acc_mps2[0] * ONE_G;
	y_acc_ms = p_acc->acc_mps2[1] * ONE_G;
	z_acc_ms = p_acc->acc_mps2[2] * ONE_G;

	// compose acceleration data
//	sprintf(mpu_data,
//		  "x_acc: %.2f, y_acc: %.2f, z_acc: %.2f || x_deg: %.2f, y_deg: %.2f, z_deg: %.2f, \r\n",
//		  x_acc_ms,
//		  y_acc_ms,
//		  z_acc_ms,
//		  p_acc->gyro_data[0],
//		  p_acc->gyro_data[1],
//		  p_acc->gyro_data[2]
//	);


	/* get orientation - tilt on x and y axis */

	// get the time of reading for rotation computations
	unsigned long t_now = HAL_GetTick();

	// set accelerometer data
	p_SensorData->x_acc = x_acc_ms;
	p_SensorData->y_acc = y_acc_ms;
	p_SensorData->z_acc = z_acc_ms;

	// these gyro values are in degrees per second
	p_SensorData->gyro_x = p_acc->gyro_data[0] - base_x_gyro;
	p_SensorData->gyro_y = p_acc->gyro_data[1] - base_y_gyro;
	p_SensorData->gyro_z = p_acc->gyro_data[2] - base_z_gyro;

	float accel_angle_y = atan(-1 * p_SensorData->x_acc/sqrt(pow(p_SensorData->y_acc, 2) + pow(p_SensorData->z_acc, 2))) * RADS_TO_DEG;
	float accel_angle_x = atan(p_SensorData->y_acc / sqrt(pow(p_SensorData->x_acc, 2) + pow(p_SensorData->z_acc, 2) )) * RADS_TO_DEG;
	float accel_angle_z = 0;

	// compute the filtered gyro angles
	float dt = (t_now - last_read_time) / 1000.0 ; // last read time in seconds
	float gyro_angle_x = p_SensorData->gyro_x * dt + last_gyro_x_angle;
	float gyro_angle_y = p_SensorData->gyro_y * dt + last_gyro_y_angle;
	float gyro_angle_z = p_SensorData->gyro_z * dt + last_gyro_z_angle;

	/* Apply complementary filter */
	p_filtered_angles->x_angle_estimate = alpha * gyro_angle_x + (1 - alpha) * (accel_angle_x);
	p_filtered_angles->y_angle_estimate = alpha * gyro_angle_y + (1 - alpha) * (accel_angle_y);
	p_filtered_angles->z_angle_estimate = gyro_angle_z; // accelerometer doesn't give z-angle

	// update the saved data with the latest values
	set_last_read_angle_data(t_now,
			p_filtered_angles->x_angle_estimate,
			p_filtered_angles->y_angle_estimate,
			p_filtered_angles->z_angle_estimate,
			gyro_angle_x,
			gyro_angle_y,
			gyro_angle_z
			);

	// construct debug data string
	sprintf(mpu_data,
			".2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f \r\n",
			accel_angle_x,
			accel_angle_y,
			accel_angle_z,
			gyro_angle_x,
			gyro_angle_y,
			gyro_angle_z,
			p_filtered_angles->x_angle_estimate,
			p_filtered_angles->y_angle_estimate,
			p_filtered_angles->z_angle_estimate
	);

	// send data over USART3 TODO: change USART channel for BluePill
	HAL_UART_Transmit(&huart3, mpu_data, sizeof(mpu_data), 10000);

	// activate file writing if needed
	#if ENABLE_FILE_WRITE
		// do the actual writing
		strncpy((char*) readBuf, mpu_data, 20); // create a string to write to sd - readBuf holds this string
		// TODO: remove magic numbers on char sizes

		UINT bytesWrote;
		fres = f_open(&fil, "data.txt", FA_WRITE | FA_OPEN_APPEND);

		// move file pointer to the end of file to append - this FatFS version has no FA_OPEN_APPEND
		f_lseek(&fil, f_size(&fil));

		if(fres == FR_OK) {
			myPrintf("[+]Open success\r\n");
		} else {
			myPrintf("[-]file open error (%i)\r\n", fres);
		}

		fres = f_write(&fil, readBuf, 60, &bytesWrote);

		if(fres == FR_OK) {
			myPrintf("[+]Wrote %i bytes to 'test.txt'!\r\n", bytesWrote);
		} else {
			myPrintf("[-]f_write error (%i)\r\n", fres);
		}

		// close the file!
		f_close(&fil);

	#endif


	HAL_Delay(10);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c1.Init.Timing = 0x00808CD2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI4_CS_Pin */
  GPIO_InitStruct.Pin = SPI4_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI4_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
