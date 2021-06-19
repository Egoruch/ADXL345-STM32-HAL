/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> /* SWO */

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
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* Flags */
uint8_t AdxlFlgInt1 = 0;
uint8_t AdxlFlgInt2 = 0;


struct AdxlCommands
{
	uint8_t	DEVID;	/*	Device ID	*/
	uint8_t	THRESH_TAP;	/*	Tap threshold	*/
	uint8_t	OFSX;	/*	X-axis offset	*/
	uint8_t	OFSY;	/*	Y-axis offset	*/
	uint8_t	OFSZ;	/*	Z-axis offset	*/
	uint8_t	DUR;	/*	Tap duration	*/
	uint8_t	Latent;	/*	Tap latency	*/
	uint8_t	Window;	/*	Tap window	*/
	uint8_t	THRESH_ACT;	/*	Activity threshold	*/
	uint8_t	THRESH_INACT;	/*	Inactivity threshold	*/
	uint8_t	TIME_INACT;	/*	Inactivity time	*/
	uint8_t	ACT_INACT_CTL;	/*	Axis enable control for activity and inactivity detection	*/
	uint8_t	THRESH_FF;	/*	Free-fall threshold	*/
	uint8_t	TIME_FF;	/*	Free-fall time	*/
	uint8_t	TAP_AXES;	/*	Axis control for single tap/double tap	*/
	uint8_t	ACT_TAP_STATUS;	/*	Source of single tap/double tap	*/
	uint8_t	BW_RATE;	/*	Data rate and power mode control	*/
	uint8_t	POWER_CTL;	/*	Power-saving features control	*/
	uint8_t	INT_ENABLE;	/*	Interrupt enable control	*/
	uint8_t	INT_MAP;	/*	Interrupt mapping control	*/
	uint8_t	INT_SOURCE;	/*	Source of interrupts	*/
	uint8_t	DATA_FORMAT;	/*	Data format control	*/
	uint8_t	DATAX0;	/*	X-Axis Data 0	*/
	uint8_t	DATAX1;	/*	X-Axis Data 1	*/
	uint8_t	DATAY0;	/*	Y-Axis Data 0	*/
	uint8_t	DATAY1;	/*	Y-Axis Data 1	*/
	uint8_t	DATAZ0;	/*	Z-Axis Data 0	*/
	uint8_t	DATAZ1;	/*	Z-Axis Data 1	*/
	uint8_t	FIFO_CTL;	/*	FIFO control	*/
	uint8_t	FIFO_STATUS;	/*	FIFO status	*/
};

struct AdxlCommands AdxlReg = {0x00, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22,
	0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D,
	0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};

enum AdxlBitNum
{
	D0,
	D1,
	D2,
	D3,
	D4,
	D5,
	D6,
	D7
};

struct AdxlData
{
	uint8_t id;
	int16_t x,y,z;

	double xg, yg, zg;

	uint8_t activity;

	uint8_t int_src;

	uint8_t raw[6];
}Adxl345;


#define SWODEBUG 1

#define SCALE_FACTOR 29.0

#define ADXL_ADR (0x53 << 1)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, uint8_t *ptr, int len)
{
	for (int DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
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
  /* USER CODE BEGIN 2 */

	uint8_t i2c_tx[2];


	/* ACCELERATION MEASURMENT */
//	/* Rate */
//	i2c_tx[0] = AdxlReg.BW_RATE;
//	i2c_tx[1] = ((1 << D3) | (1 << D1));
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Range 16g */
//	i2c_tx[0] = AdxlReg.DATA_FORMAT;
//	i2c_tx[1] = ((1 << D0) | (1 << D1));
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Measure */
//	i2c_tx[0] = AdxlReg.POWER_CTL;
//	i2c_tx[1] = (1 << D3);
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);



	/* ACTIVITY */
//
//	/* Activity Axes */
//	i2c_tx[0] = AdxlReg.ACT_INACT_CTL;
//	i2c_tx[1] = ((1 << D7) | (1 << D6) | (1 << D5) | (1 << D4));
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Threshold Activity */
//	i2c_tx[0] = AdxlReg.THRESH_ACT;
//	i2c_tx[1] = 10;
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Int Map */
//	i2c_tx[0] = AdxlReg.INT_MAP;
//	i2c_tx[1] = ~(1 << D4);
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Interrupt Enable */
//	i2c_tx[0] = AdxlReg.INT_ENABLE;
//	i2c_tx[1] = (1 << D4);
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Measure */
//	i2c_tx[0] = AdxlReg.POWER_CTL;
//	i2c_tx[1] = (1 << D3);
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);



	/* FREE-FALL */

//	/* Free-fall threshold */
//	i2c_tx[0] = AdxlReg.THRESH_FF;
//	i2c_tx[1] = 3;
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Free-fall time */
//	i2c_tx[0] = AdxlReg.TIME_FF;
//	i2c_tx[1] = 2;
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Int Map */
//	i2c_tx[0] = AdxlReg.INT_MAP;
//	i2c_tx[1] = ~(1 << D2);
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Interrupt Enable */
//	i2c_tx[0] = AdxlReg.INT_ENABLE;
//	i2c_tx[1] = (1 << D2);
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Measure */
//	i2c_tx[0] = AdxlReg.POWER_CTL;
//	i2c_tx[1] = (1 << D3);
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	uint8_t ActCnt = 0;



//	/* DOUBLE TAP */
//
//	/* Tap Axes */
//	i2c_tx[0] = AdxlReg.TAP_AXES;
//	i2c_tx[1] = ((1 << D2) | (1 << D1) | (1 << D0));
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Threshold Tap */
//	i2c_tx[0] = AdxlReg.THRESH_TAP;
//	i2c_tx[1] = 60;
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Tap Duration */
//	i2c_tx[0] = AdxlReg.DUR;
//	i2c_tx[1] = 40;
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Tap Latency */
//	i2c_tx[0] = AdxlReg.Latent;
//	i2c_tx[1] = 80;
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Tap Window */
//	i2c_tx[0] = AdxlReg.Window;
//	i2c_tx[1] = 200;
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Interrupt Enable */
//	i2c_tx[0] = AdxlReg.INT_ENABLE;
//	i2c_tx[1] = (1 << D5);
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Interrupt Map */
//	i2c_tx[0] = AdxlReg.INT_MAP;
//	i2c_tx[1] = ~(1 << D5);
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);
//
//	/* Measure */
//	i2c_tx[0] = AdxlReg.POWER_CTL;
//	i2c_tx[1] = (1 << D3);
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)i2c_tx, 2, 1000);



//	/* DEVICE ID */
//
//	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, &AdxlReg.DEVID, 1, 1000);
//	HAL_I2C_Master_Receive(&hi2c1, ADXL_ADR, (uint8_t*)&Adxl345.id, 1, 1000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_I2C_Mem_Read(&hi2c1, ADXL_ADR, AdxlReg.INT_SOURCE, 1, &Adxl345.int_src, 1, 100);
	  if(Adxl345.int_src & (1 << D2)) /* D2 - FREEFALL, D5 - DOUBLE TAP, D4 - ACTIVITY*/
	  {
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);


		  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

		  Adxl345.int_src = 0;
	  }



//	  if(AdxlFlgInt1)
//	  {
//		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//
//		  AdxlFlgInt1 = 0;
//
//		  //printf("%d\r\n", ++ActCnt);
//
//		  HAL_I2C_Mem_Read(&hi2c1, ADXL_ADR, AdxlReg.INT_SOURCE, 1, &Adxl345.int_src, 1, 100);
//	  }
//


//
//	  if(AdxlFlgInt2)
//	  {
//		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//
//		  //printf("Free-fall detected!\r\n");
//
//		  HAL_I2C_Mem_Read(&hi2c1, ADXL_ADR, (uint16_t)0x30, 1, (uint8_t*)&Adxl345.int_src, 1, 100);
//
//		  AdxlFlgInt2 = 0;
//	  }


	/* XYZ ACCELERATION */

	HAL_I2C_Master_Transmit(&hi2c1, ADXL_ADR, (uint8_t*)&AdxlReg.DATAX0, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, ADXL_ADR, (uint8_t*)&Adxl345.raw, 6, 100);


	Adxl345.x = ((int16_t)(Adxl345.raw[1] << 8) | Adxl345.raw[0]);
	Adxl345.y = ((int16_t)(Adxl345.raw[3] << 8) | Adxl345.raw[2]);
	Adxl345.z = ((int16_t)(Adxl345.raw[5] << 8) | Adxl345.raw[4]);


	Adxl345.xg = (SCALE_FACTOR*Adxl345.x)/1000;
	Adxl345.yg = (SCALE_FACTOR*Adxl345.y)/1000;
	Adxl345.zg = (SCALE_FACTOR*Adxl345.z)/1000;

#if(SWODEBUG)
//	printf("Adxl345.xg = %.3f\r\n", Adxl345.xg);
#endif


	HAL_Delay(10);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == INT1_Pin)
	{
		/* Set Flag */
	    AdxlFlgInt1 = 255;
	}

	if(GPIO_Pin == INT2_Pin)
	{
		/* Set Flag */
	    AdxlFlgInt2 = 255;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
