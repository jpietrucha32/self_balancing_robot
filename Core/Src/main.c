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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_PI			3.14159
#define STEPS_PER_REV	200
#define MICROSTEP_NUM	8
#define	XT_TIM_CLK		80000000
#define XT_TIM_PSC		79
#define LSM303D_ADDR	0x3a
#define L3GD20H_ADDR	0xd6
#define	PULSE_TIM_16	htim16
#define	PULSE_TIM_CH1	TIM_CHANNEL_1
#define	PULSE_TIM_1		htim1
#define	PULSE_TIM_CH3	TIM_CHANNEL_3
#define LSM303D_WHO_AM_I			0x0f
#define LSM303D_CTRL1				0x20
#define LSM303D_CTRL2				0x21
#define LSM303D_CTRL5				0x24
#define CTRL4						0x23
#define LSM303D_OUT_X_A				0x28
#define LSM303D_OUT_Y_A				0x2a
#define LSM303D_OUT_Z_A				0x2c
#define GYRO_OFFSET					-1.1109
#define Kp							14.5
#define Ki							0.0002
#define Kd							18.7
#define setpoint					0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float y_tilt=0;
float acc_tilt_total = 0;
int iteration_num=0;
float average_y_tilt=0;
float tilt_integral=0;
float last_tilt=0;
float actual_tilt=0;
long int actual_time=0;
long int last_time=0;
float speed=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Sending data by UART on PC
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}

//Setting speed of left motor
void set_stepper_L_motor_speed(float rpm){
	if(rpm > 0)
		{
		//Changing timer No.16 timing
			uint16_t arr_val = (uint16_t)(XT_TIM_CLK / (((XT_TIM_PSC+1) * rpm * STEPS_PER_REV * MICROSTEP_NUM) / 60) - 1);
			__HAL_TIM_SET_AUTORELOAD(&PULSE_TIM_16, arr_val);
			__HAL_TIM_SET_COMPARE(&PULSE_TIM_16, PULSE_TIM_CH1, arr_val-1);
		}
		else
		{
			//If speed 0 set timing to 0
			__HAL_TIM_SET_COMPARE(&PULSE_TIM_16, PULSE_TIM_CH1, 0);
		}
}

//Setting speed of right motor
void set_stepper_R_motor_speed(float rpm){
	if(rpm > 0)
		{
		//Changing timer No.1 timing
			uint16_t arr_val = (uint16_t)(XT_TIM_CLK / (((XT_TIM_PSC+1) * rpm * STEPS_PER_REV * MICROSTEP_NUM) / 60) - 1);
			__HAL_TIM_SET_AUTORELOAD(&PULSE_TIM_1, arr_val);
			__HAL_TIM_SET_COMPARE(&PULSE_TIM_1, PULSE_TIM_CH3, arr_val-1);
		}
		else
		{
			//If speed 0 set timing to 0
			__HAL_TIM_SET_COMPARE(&PULSE_TIM_1, PULSE_TIM_CH3, 0);
		}
}

//Reading value from chosen register
uint8_t lsm_read_reg(uint8_t reg, uint8_t dev)
{
	uint8_t value = 0;

	HAL_I2C_Mem_Read(&hi2c1, dev, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);

	return value;
}

//Writing value to register

void lsm_write_reg(uint8_t reg, uint8_t value, uint8_t dev)
{
	HAL_I2C_Mem_Write(&hi2c1, dev, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
}


int16_t lsm_read_value(uint8_t reg, uint8_t dev)
{
	int16_t value = 0;

	HAL_I2C_Mem_Read(&hi2c1, dev, reg | 0x80, 1, (uint8_t*)&value, sizeof(value), HAL_MAX_DELAY);

	return value;
}

//Init work of accelerometer
int LSM303D_init(){
	printf("Wyszukiwanie akcelerometru...\r\n");
	uint8_t who_am_i = lsm_read_reg(0xf,LSM303D_ADDR);
	if (who_am_i == 0x49) {
		printf("Znaleziono akcelerometr LSM303D\r\n");
		return 1;
	 }
	else{
		printf("Niepoprawna odpowiedź układu (0x%02X)\r\n", who_am_i);
		return 0;
	 }
}

//Init work of gyroscope
int L3GD20H_init(){
	printf("Wyszukiwanie zyroskopu...\r\n");
	uint8_t who_am_i = lsm_read_reg(0xf,L3GD20H_ADDR);
	if (who_am_i == 0xD7) {
		printf("Znaleziono zyroskop L3GD20H\r\n");
		return 1;
	 }
	else{
		printf("Niepoprawna odpowiedź układu (0x%02X)\r\n", who_am_i);
		return 0;
	 }
}

//Function to checking if all sensor started successfully
int sensors_run_correctly(){
	int LSM303D_status = LSM303D_init();
	int L3GD20H_status = L3GD20H_init();
	if(LSM303D_status && L3GD20H_status) return 1;
	else return 0;
}

//Init time interrupts if  sensors started working
void activate_app(){
	if(sensors_run_correctly()){
		lsm_write_reg(LSM303D_CTRL1, 0x40|0x07, LSM303D_ADDR);
		lsm_write_reg(LSM303D_CTRL1, 0xCF, L3GD20H_ADDR);
		lsm_write_reg(CTRL4, 0x10, L3GD20H_ADDR);
		printf("Start aplikacji...\r\n");
		HAL_TIM_Base_Start_IT(&htim6);
	}
	else printf("Brak poloaczenia z sensorami...\r\n");
}

//Reading values from register of accelerometer
float calc_acc_tilt(){
	int16_t a_x = lsm_read_value(LSM303D_OUT_X_A, LSM303D_ADDR);
 	int16_t a_z = lsm_read_value(LSM303D_OUT_Z_A, LSM303D_ADDR);
 	float tilt_a_x = a_x * 2.0f / 32678.0f;
 	float tilt_a_z = a_z * 2.0f / 32678.0f;
 	float alpha = atan2f(tilt_a_z, tilt_a_x);
 	return (alpha * 180.0f / M_PI - 90.0f);
}

//Implementation of Kalman filter
struct Kalman{
	float Q_angle;
	float Q_bias;
	float R_measure;
	float angle;
	float bias;
	float rate;
	float P[2][2];
};

//Init Kalman filter
void init_Kalman(struct Kalman* kalman){
	kalman -> P[0][0] = 0.0f;
	kalman -> P[0][1] = 0.0f;
	kalman -> P[1][0] = 0.0f;
	kalman -> P[1][1] = 0.0f;

	kalman -> Q_angle = 0.0009f;
	kalman -> Q_bias = 0.1f;
	kalman -> R_measure = 3.0f;

	kalman -> angle = 0.0f;
	kalman -> bias = 9.0f;
}

//Function to estimate tilt of robot by Kalman filter
float getAngle(struct Kalman *kalman ,float newAngle, float newRate, float dt){
	kalman -> rate = newRate  - kalman -> bias;
	kalman -> angle += kalman -> rate * dt;

	kalman -> P[0][0] += dt * (dt*kalman -> P[1][1] - kalman -> P[0][1] - kalman -> P[1][0] + kalman -> Q_angle);
	kalman -> P[0][1] -= dt * kalman -> P[1][1];
	kalman -> P[1][0] -= dt * kalman -> P[1][1];
	kalman -> P[1][1] += dt * kalman -> Q_bias;

	float S = kalman -> P[0][0] + kalman -> R_measure;
	float K[2];
	K[0] = kalman -> P[0][0] / S;
	K[1] = kalman -> P[1][0] / S;

	float y = newAngle - kalman -> angle;
	kalman -> angle += K[0] * y;
	kalman -> bias += K[1] * y;

	float P00_temp = kalman -> P[0][0];
	float P01_temp = kalman -> P[0][1];

	kalman -> P[0][0] -= K[0] * P00_temp;
	kalman -> P[0][1] -= K[0] * P01_temp;
	kalman -> P[1][0] -= K[1] * P00_temp;
	kalman -> P[1][1] -= K[1] * P01_temp;

	return kalman->angle;
}

//Saving actual angle to struct
void setAngle(struct Kalman *kalman, float angle){
	kalman -> angle = angle;
}

//Init new Kalman variable
struct Kalman kalman;
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
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //Initialization of PWM generators for both stepper motors
  HAL_TIM_Base_Start(&htim16);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  TIM_CCxChannelCmd(htim16.Instance, TIM_CHANNEL_1, TIM_CCxN_ENABLE);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_ENABLE);
  //activation of app
  activate_app();
  float acc_tilt = calc_acc_tilt();
  init_Kalman(&kalman);
  setAngle(&kalman, acc_tilt);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x10909CEC;
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
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0xffff-2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 79;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Y_DIR_Pin|LD3_Pin|Z_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Y_DIR_Pin LD3_Pin Z_DIR_Pin */
  GPIO_InitStruct.Pin = Y_DIR_Pin|LD3_Pin|Z_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : X_DIR_Pin */
  GPIO_InitStruct.Pin = X_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(X_DIR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//Determining pid control every 1ms
 	  if(htim->Instance == TIM6){
 		 //Calculate the slope of robot with accelerometer
 		 float acc_angle = calc_acc_tilt();
 		 //Calculate the slope of robot with gyroscope
 		 int16_t g_y = lsm_read_value(LSM303D_OUT_Y_A, L3GD20H_ADDR);
 		 float omega = g_y*0.0175f;
 		 //Fusion of accelerometer and gyroscope data by Kalman filter
 		 float tilt = getAngle(&kalman, acc_angle, omega, 0.001);
 		 printf("%.2f\r\n",tilt);
 		 //Calculating the integral of the error
 		 if(!isnan(tilt) && fabs(tilt_integral) < 3500) tilt_integral += tilt;
 		 //Calculate speed of stepper motors by PID regulator
 		 speed = (fabs(tilt-setpoint)*Kp) + (fabs(tilt-last_tilt)*Ki) + fabs(tilt_integral * Kd);
 		 //Set speed of motors
 		 set_stepper_L_motor_speed(speed);
 		 set_stepper_R_motor_speed(speed);
 		 //Set direction of motors
 		 if(tilt>0){
 			HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_RESET);
 			HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_SET);
 			 }
 		 else{
 			HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, GPIO_PIN_SET);
 		    HAL_GPIO_WritePin(Y_DIR_GPIO_Port, Y_DIR_Pin, GPIO_PIN_RESET);
 		 }
 		 //Saving last tilt to get derivate
 		 last_tilt = tilt;
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
