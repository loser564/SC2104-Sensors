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
#include "MPU6050.h"
#include "oled.h"


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
 I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t IMU_ADDR = 0x68<<1;
uint32_t millis0ld, millisNow;

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
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  	 IMU_Data imu;
  	char sbuf[10];
  	OLED_Init();
  	float pitch_acc,roll_acc;
  	float dt;
  	uint8_t status = IMU_Initialise(&imu, &hi2c2, &huart3);
  	uint32_t miiliS0ld, millisNow;

  	IMU_GyroRead(&imu);
  	IMU_AccelRead(&imu);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  IMU_GyroRead(&imu);
	  IMU_AccelRead(&imu);
	  /*for(int i=0;i<3;i++){
	  		sprintf(sbuf, "%5.2f, ", imu.acc[i]);
	  		HAL_UART_Transmit(&huart3, sbuf, strlen(sbuf), HAL_MAX_DELAY);
	  	}
	  	*/


	  	pitch_acc =atan2(imu.acc[0],imu.acc[2]);
	  	pitch_acc=-pitch_acc*57.3;
	  	roll_acc = atan2(imu.acc[1],imu.acc[2]);
	  	roll_acc = roll_acc * 57.3; // radian to degree
	  	/*
	  	sprintf(sbuf, "%5.2f ", imu.acc[0]);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 8, HAL_MAX_DELAY);
	  	OLED_ShowString(10,5,sbuf);
	  	OLED_Refresh_Gram();

	  	sprintf(sbuf, "%5.2f, ", imu.acc[1]);
	  		  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 8, HAL_MAX_DELAY);
	  		  	OLED_ShowString(10,15,sbuf);
	  		  	OLED_Refresh_Gram();

	  	sprintf(sbuf, "%5.2f, ", imu.acc[2]);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 8, HAL_MAX_DELAY);
	  	OLED_ShowString(10,25,sbuf);
	  	OLED_Refresh_Gram();



	  	sprintf(sbuf,"%5.2f ", pitch_acc);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	OLED_ShowString(30,5,sbuf);
	  	OLED_Refresh_Gram();


	  	sprintf(sbuf,"%5.2f ", roll_acc);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	OLED_ShowString(30,15,sbuf);
	  	OLED_Refresh_Gram();
	  	//HAL_UART_Transmit(&huart3, "\r\n", 2, HAL_MAX_DELAY);
		*/





	  	millisNow = HAL_GetTick();
	  	dt = (millisNow - millis0ld)*0.001;
	  	millis0ld = millisNow;
	  	float groll_now,groll_prev,gpitch_now,gpitch_prev, gyaw_prev, gyaw_now;
	  	float CF_pitch,CF_roll,KF_pitch,KF_roll,KG_pitch,KG_roll;
	  	float var_groll, var_gpitch;
	  	int Var_gyro = 16;
	  	int Var_acc = 9;


	  	groll_prev = groll_now;
	  	groll_now = groll_prev + imu.gyro[0]*dt;
	  	gpitch_prev = gpitch_now;
	  	gpitch_now = gpitch_prev + imu.gyro[1]*dt;
	  	gyaw_prev = gyaw_now;
	  	gyaw_now = gyaw_prev + imu.gyro[2]*dt;


	  	/*
	  	sprintf(sbuf, "%5.2f ", groll_now);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	OLED_ShowString(30,35,sbuf);
	  	OLED_Refresh_Gram();


	  	sprintf(sbuf, "%5.2f,",-gpitch_now);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	OLED_ShowString(30,45,sbuf);
	  	OLED_Refresh_Gram();


	  	sprintf(sbuf, "%5.2f ", gyaw_now);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	OLED_ShowString(30,55,sbuf);
	  	OLED_Refresh_Gram();
	  	//HAL_UART_Transmit(&huart3, "\r\n", 2, HAL_MAX_DELAY);

		*/

	  	// Lab 3 starts here yay

	  	// accelerometer angle values
	  	// pitch_acc
	  	// roll_acc
	  	int a;
	  	a= 0.9;
	  	/*
	  	CF_pitch = (1-a)*(pitch_acc) + a*(-gpitch_now);
	  	sprintf(sbuf, "%5.2f,",CF_pitch);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	OLED_ShowString(10,15,sbuf);
	  	OLED_Refresh_Gram();


	  	CF_roll = (1-a)*(roll_acc) + a*(groll_now);
	  	sprintf(sbuf, "%5.2f,",CF_roll);
	  	HAL_UART_Transmit(&huart3, sbuf, 6, HAL_MAX_DELAY);
	  	HAL_UART_Transmit(&huart3, "\r\n", 2, HAL_MAX_DELAY);
	  	OLED_ShowString(10,25,sbuf);
	  	OLED_Refresh_Gram();
	  	*/



	  	KF_roll = KF_roll + imu.gyro[0]*dt;
	  	KF_pitch = KF_pitch + imu.gyro[1]*dt;

	  	var_groll = var_groll + dt*dt*Var_gyro;
	  	var_gpitch = var_gpitch + dt*dt*Var_gyro;

	  	KG_roll = var_groll/(var_groll + Var_acc);
	  	KG_pitch = var_gpitch/(var_gpitch + Var_acc);

	  	KF_roll = KF_roll + KG_roll*(roll_acc - KF_roll);
	  	KF_pitch = KF_pitch + KG_pitch*(pitch_acc - KF_pitch);

	  	var_groll = (1-KG_roll)*var_groll;
	  	var_gpitch = (1-KG_pitch)*var_gpitch;


	  	//pitch
	  	/*

	  	sprintf(sbuf, "%5.2f,",-gpitch_now);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	OLED_ShowString(10,15,sbuf);
	  	OLED_Refresh_Gram();

	  	sprintf(sbuf,"%5.2f,", pitch_acc);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	OLED_ShowString(30,25,sbuf);
	  	OLED_Refresh_Gram();


	  	OLED_ShowString(10,25,sbuf);
	  	OLED_Refresh_Gram();
	  	CF_pitch = (1-a)*(pitch_acc) + a*(-gpitch_now);
	  	sprintf(sbuf, "%5.2f,",CF_pitch);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	OLED_ShowString(10,35,sbuf);
	  	OLED_Refresh_Gram();


	  	sprintf(sbuf, "%5.2f,",KF_pitch);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	//HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	HAL_UART_Transmit(&huart3, "\r\n", 2, HAL_MAX_DELAY);

	  	OLED_ShowString(10,45,sbuf);
	  	OLED_Refresh_Gram();
	  	//HAL_Delay(500);
	  	 */


	  	//roll


	  	sprintf(sbuf,"%5.2f,", roll_acc);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	OLED_ShowString(30,15,sbuf);
	  	OLED_Refresh_Gram();

	  	sprintf(sbuf, "%5.2f,", groll_now);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	OLED_ShowString(30,25,sbuf);
	  	OLED_Refresh_Gram();

	  	CF_roll = (1-a)*(roll_acc) + a*(groll_now);
	  	sprintf(sbuf, "%5.2f,",CF_roll);
	  	HAL_UART_Transmit(&huart3, sbuf, 6, HAL_MAX_DELAY);
	  	OLED_ShowString(10,35,sbuf);
	  	OLED_Refresh_Gram();

	  	sprintf(sbuf, "%5.2f,",KF_roll);
	  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);

	  	OLED_ShowString(10,45,sbuf);
	  	OLED_Refresh_Gram();
	  	HAL_UART_Transmit(&huart3, "\r\n", 2, HAL_MAX_DELAY);



	  	//kalmann gain
//	  	sprintf(sbuf, "%5.2f,",KF_roll);
//	  		  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
//	  		  	OLED_ShowString(10,15,sbuf);
//	  		  	OLED_Refresh_Gram();
//
//	  		  	sprintf(sbuf,"%5.2f,", KG_roll);
//	  		  	HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
//	  		  	OLED_ShowString(30,25,sbuf);
//	  		  	OLED_Refresh_Gram();
//
//	  		  HAL_UART_Transmit(&huart3, "\r\n", 2, HAL_MAX_DELAY);



	  	//HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, 6, HAL_MAX_DELAY);
	  	//HAL_Delay(1000);









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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.ClockSpeed = 100000;
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DC_Pin|RESET__Pin|SDIN_Pin|SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin RESET__Pin SDIN_Pin SCLK_Pin */
  GPIO_InitStruct.Pin = DC_Pin|RESET__Pin|SDIN_Pin|SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
