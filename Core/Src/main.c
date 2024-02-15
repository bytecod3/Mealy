/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "MPU6050.h"
#include "motor.h"
#include "pid.h"
#include "RGB.h"
#include "ssd1306.h"
#include "fonts.h"
#include "splash_screen.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ONE_G 				(9.81) 	// acceleration due to gravity
#define LOOP_DELAY 			(10)	// infinite loop delay
#define SPLASH_DELAY 		3000 	// interval when displaying splash screen

// PID defines
#define MOTOR_REFERENCE 200	// define the motor reference speed
#define motor_reference_angle 0 // the imu is placed such that a change in y axis drives the tilt
//float sample_rate = 0.000125; // 0.125 ms TODO: use system clock to get sample rate
uint16_t sampling_frequency = 8000000;

// motor defines
#define LEFT_DC_MOTOR 0
#define RIGHT_DC_MOTOR 1

// create motor PID instances
// left motor PID instance
pid_instance left_motor_pid_instance;
pid_instance* p_left_motor_instance = &left_motor_pid_instance;

// right motor PID instance
pid_instance right_motor_pid_instance;
pid_instance* p_right_motor_instance = &right_motor_pid_instance;

// RGB LED variables
// create an RGB LED instance
RGB_instance mealy_rgb;
RGB_instance* p_mealy_rgb = &mealy_rgb;

// a color struct to hold different colors
RGB_color mealy_colors;
RGB_color* p_mealy_colors = &mealy_colors;
uint8_t seconds = 0;
uint32_t last_time = 0;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char msg[20] = "";
char mpu_data[100];
char data[40] = ""; // sd card test char array

// create accelerometer struct
MPU6050 acc;
MPU6050* p_acc = &acc; // pointer to accelerometer
float x_acc_ms, y_acc_ms, z_acc_ms; // acceleration converted to m/s^2

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
unsigned long 	last_read_time = 0;
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
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void myPrintf(const char *fmt, ...); // function to send formatted data over serial
void setDutyCycle(int dutyCycle); // function to set PWM duty cycle

// OLED
// display on start screen sequence before I start running
void sayHello();

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
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);
}

void sayHello() {

	// display logo bitmap
	SSD1306_Clear();
	SSD1306_DrawBitmap(0,0,splash_screen,128,64,1);
	SSD1306_UpdateScreen();

	HAL_Delay(SPLASH_DELAY);

	// hello world message
	SSD1306_Clear();
	SSD1306_GotoXY (30, 20);
	SSD1306_Puts ("Hello", &Font_11x18, 1);
	SSD1306_GotoXY (20, 40);
	SSD1306_Puts ("world :)", &Font_11x18, 1);
	SSD1306_UpdateScreen(); //display

	HAL_Delay(SPLASH_DELAY);

	// introduction
	SSD1306_Clear();
	SSD1306_GotoXY(15, 16);
	SSD1306_Puts("My name is", &Font_11x18, 1);
	SSD1306_GotoXY(25, 35);
	SSD1306_Puts("Mealy...", &Font_11x18, 1);
	SSD1306_UpdateScreen(); //display

	HAL_Delay(SPLASH_DELAY);

	SSD1306_Clear();
	SSD1306_DrawBitmap(0,0,powered_by,128,64,1);
	SSD1306_UpdateScreen();

	HAL_Delay(SPLASH_DELAY);

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // start motors
  motor_init(LEFT_DC_MOTOR);
  motor_init(RIGHT_DC_MOTOR);
  motor_start(LEFT_DC_MOTOR, CW, 0);
  motor_start(RIGHT_DC_MOTOR, CW, 0);

  // set pID gains
  set_pid(p_left_motor_instance, 40, 600, 0.8);
  set_pid(p_right_motor_instance, 40, 600, 0.8);

  set_pid(p_left_motor_instance, 40, 600, 0.8);
  set_pid(p_right_motor_instance, 40, 600, 0.8);

  /*-------------------------INERTIAL MEASUREMENT UNIT FUNCTIONS--------------------------------*/
  MPU6050_Initialise(&acc, &hi2c1); // TODO: check init status here

  // calibrate the sensor
  callibrate_sensor();

  // initialise the sensor angles to known values
  set_last_read_angle_data(HAL_GetTick(), 0, 0, 0, 0, 0, 0);

  //-------------RGB---------------------

  // timer to generate 1 second pulses - used to run RGB time state machine
	HAL_TIM_Base_Start_IT(&htim3);
	TIM3->CCR1 = 500; // set timer 3 duty cycle to 50%

	// set parameters for RGB LED
	// for setting fade interval
	last_time = HAL_GetTick();

	RGB_set_parameters(p_mealy_rgb, &htim1, last_time);

	RGB_init(p_mealy_rgb);

	RGB_color_code white = WHITE;
	RGB_color_code green = GREEN;
	RGB_color_code red = RED;
	RGB_color_code blue = BLUE;

	p_mealy_colors->red_value = 0;
	p_mealy_colors->green_value = 0;
	p_mealy_colors->blue_value = 255;

	// initalise OLED screen
	SSD1306_Init();

	sayHello();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

	  	/*-------------- get orientation - tilt on x and y axis */

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

	    // compute angles from the accelerometer
	  	float accel_angle_y = atan(-1 * p_SensorData->x_acc/sqrt(pow(p_SensorData->y_acc, 2) + pow(p_SensorData->z_acc, 2))) * RADS_TO_DEG;
	  	float accel_angle_x = atan(p_SensorData->y_acc / sqrt( pow(p_SensorData->x_acc, 2) + pow(p_SensorData->z_acc, 2) )) * RADS_TO_DEG;
	  	float accel_angle_z = 0;

	  	float dt = (t_now - last_read_time) / 1000.0 ; // last read time in seconds

	  	// compute the filtered gyro angles
	  	float gyro_angle_x = p_SensorData->gyro_x * dt + last_x_angle;
	  	float gyro_angle_y = p_SensorData->gyro_y * dt + last_y_angle;
	  	float gyro_angle_z = p_SensorData->gyro_z * dt + last_z_angle;

	  	/* Apply complementary filter */
	  	p_filtered_angles->x_angle_estimate = alpha * gyro_angle_x + (1 - alpha) * (accel_angle_x);
	  	p_filtered_angles->y_angle_estimate = alpha * gyro_angle_y + (1 - alpha) * (accel_angle_y);
	  	p_filtered_angles->z_angle_estimate = gyro_angle_z; // accelerometer doesn't give z-angle

	  	// update the saved data with the latest values
	  	set_last_read_angle_data(
	  			t_now,
	  			p_filtered_angles->x_angle_estimate,
	  			p_filtered_angles->y_angle_estimate,
	  			p_filtered_angles->z_angle_estimate,
	  			gyro_angle_x,
	  			gyro_angle_y,
	  			gyro_angle_z
	  			);

	  	// according to how i've mounted the MPU, i'm interested in x-axis angle
//	  	sprintf(mpu_data,
//	  			"x: %.2f, y: %.2f \r\n",
//				p_filtered_angles->x_angle_estimate,
//				p_filtered_angles->y_angle_estimate
//	  			);

//	  	sprintf(mpu_data, "%d\r\n", seconds);

	  	// run motors

	  	apply_pid(p_left_motor_instance, motor_reference_angle - p_filtered_angles->x_angle_estimate,  sampling_frequency);
	  	apply_pid(p_right_motor_instance, motor_reference_angle - p_filtered_angles->x_angle_estimate, sampling_frequency);

	  	if(p_left_motor_instance->output < 0 ) {

	  		motor_set_dir(LEFT_DC_MOTOR, CW);
	  		motor_set_speed(LEFT_DC_MOTOR, p_left_motor_instance->output);
	  	}

	  	if (p_right_motor_instance->output < 0 ) {
	  		motor_set_dir(RIGHT_DC_MOTOR, CW);
	  		motor_set_speed(RIGHT_DC_MOTOR, p_right_motor_instance->output);
	  	}

	  	if(p_left_motor_instance->output > 0 ) {

			motor_set_dir(LEFT_DC_MOTOR, CCW);
			motor_set_speed(LEFT_DC_MOTOR, p_left_motor_instance->output);
		}

		if (p_right_motor_instance->output > 0 ) {
			motor_set_dir(RIGHT_DC_MOTOR, CCW);
			motor_set_speed(RIGHT_DC_MOTOR, p_right_motor_instance->output);
		}

		//---------------RGB-----------------
		// change pulse color after set time interval
	  if(seconds < 15) {
		  // pulse a different color
		  // fade white
		  pulse(p_mealy_rgb, red, last_time);

	  } else if(seconds < 30) {
		  // fade green
		  pulse(p_mealy_rgb, green, last_time);
	  } else if(seconds > 30) {
		  // fade red
		  pulse(p_mealy_rgb, white , last_time);
	  }

	  if(seconds == 60) {
		  seconds = 0;
	  }


	  // send data over USART3 TODO: change USART channel for BluePill
//	  HAL_UART_Transmit(&huart2, (uint8_t*)mpu_data, sizeof(mpu_data), 100);
//	  HAL_UART_Transmit(&huart1, (uint8_t*) "Sending data...", strlen("Sending data..."), 100);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 8000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	seconds++;
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
