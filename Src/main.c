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
#include "stm32_ds3231.h"
#include "stm32_ds3231.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define numlines 2
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
I2C_HandleTypeDef hi2c1;
uint8_t menuFlag = 0;
uint8_t buttonFlags = 0;
uint16_t tempData = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
static void callButton(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
_RTC rtc = {
    .Year = 0, .Month = 3, .Date = 1,
    .DaysOfWeek = SUNDAY,
    .Hour = 2, .Min = 8, .Sec = 0
};

uint8_t regVal;
float rtcTemp;

void lcdSend(uint8_t isCommand, uint8_t data) {
	//HAL_Delay(25);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, isCommand ? 0 : 1); //RS
    HAL_Delay(1);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (data >> 7) & 1); //D7
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (data >> 6) & 1); //D6
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (data >> 5) & 1); //D5
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (data >> 4) & 1); //D4


        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1); //E
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (data >> 3) & 1);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (data >> 2) & 1);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (data >> 1) & 1);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (data >> 0) & 1);

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
      HAL_Delay(1);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);

}

void lcdCommand(uint8_t cmd) {
    lcdSend(1, cmd);
}

void lcdChar(const char chr) {
    lcdSend(0, (uint8_t)chr);
}

void lcdString(const char* str) {
    while(*str != '\0') {
        lcdChar(*str);
        str++;
    }
}
void lcdInt(int value) {
	char buf[17];
	sprintf(buf, "%d", value);
	lcdString(buf);
}

void lcdSetCursor(uint8_t x, uint8_t y) {
	/*
	if (0 <= x && x <= 100) {
		lcdCommand(0x80 | x);
	}
	if (0 <= y && y <= 1) {
		lcdCommand(0x80 | y);
	}
	*/

  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( y >= numlines ) {
    y = numlines-1;    // we count rows starting w/0
  }
  
  lcdCommand(LCD_SETDDRAMADDR | (x + row_offsets[y]));

}
void checkDate() {
	if(rtc.Month == 1 || rtc.Month == 3 || rtc.Month == 5 || rtc.Month == 7 || rtc.Month == 8 ||  rtc.Month == 10 || rtc.Month == 12) {
		if(tempData <= 31) rtc.Date = tempData;
		else tempData = 1;
}
	else if (rtc.Month == 4 || rtc.Month == 6 || rtc.Month == 9 || rtc.Month == 11) {
		if(tempData <= 30) rtc.Date = tempData;
		else tempData = 1;
	}
	else if (rtc.Month == 2) {
		if(rtc.Year % 4 == 0) {
			if(tempData <= 29) rtc.Date = tempData;
		else tempData = 1;
		}
		else if(rtc.Year % 4 != 0) {
			if(tempData <= 28) rtc.Date = tempData;
		else tempData = 1;
		}
	}
}
void setTime() {
	while(1) {
	if(menuFlag == 0) return;
		callButton();
		switch(menuFlag) {
			case 1:
			lcdSetCursor(0,0);
			lcdString("Set year: ");
			lcdInt(tempData);
			rtc.Year = tempData - 2000;
			break;
			case 2:
			lcdSetCursor(0,0);
			lcdString("Set month: ");
						if(tempData <= 12) rtc.Month = tempData;
			else tempData = 1;
			if(tempData < 10) {
			lcdChar('0');
lcdInt(tempData);
		}
		else 	lcdInt(tempData);
			break;
			case 3:
					lcdSetCursor(0,0);
			lcdString("Set date: ");
			checkDate();
						if(tempData < 10) {
			lcdChar('0');
lcdInt(tempData);
		}
		else 	lcdInt(tempData);
			break;
			case 4:
					lcdSetCursor(0,0);
			lcdString("Set hour: ");
			if(tempData <= 23) rtc.Hour = tempData;
			else tempData = 1;
			if(tempData < 10) {
			lcdChar('0');
lcdInt(tempData);
		}
		else 	lcdInt(tempData);
			break;
			case 5:
					lcdSetCursor(0,0);
			lcdString("Set minute: ");
			if(tempData <= 59) rtc.Min = tempData;
			else tempData = 1;
			if(tempData < 10) {
			lcdChar('0');
lcdInt(tempData);
		}
		else 	lcdInt(tempData);
			break;
			case 6:
			lcdSetCursor(0,0);
			lcdString("Set second: ");
			if(tempData <= 59) rtc.Sec = tempData;
			else tempData = 1;
			if(tempData < 10) {
			lcdChar('0');
lcdInt(tempData);
		}
		else 	lcdInt(tempData);
			break;
			default:
			DS3231_SetTime(&rtc);
			menuFlag = 0;
			break;
	}
}
	}
void check1() {
tempData++;
}

void check2() {
lcdCommand(0x01); 
if(menuFlag == 0) {
	menuFlag++;
	HAL_Delay(10);
					tempData = 2000;
	setTime();
}
	else {
		tempData = 1;
		menuFlag++;
		HAL_Delay(10);
	}
}
void callButton() {
  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_13) == GPIO_PIN_SET && ((buttonFlags >> 0) & 1) == 1) {
   buttonFlags &= ~(1 << 0);
  }
   if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_13) == GPIO_PIN_RESET && ((buttonFlags >> 0) & 1) == 0) {
   buttonFlags |= (1 << 0);
     check1();
  }
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_14) == GPIO_PIN_SET && ((buttonFlags >> 1) & 1) == 1) {
    buttonFlags &= ~(1 << 1);
  }
   if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_14) == GPIO_PIN_RESET && ((buttonFlags >> 1) & 1) == 0) {
  buttonFlags |= (1 << 1);
     check2();
  }
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
buttonFlags = (1 | (1 << 1));
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
	  DS3231_Init(&hi2c1);
 // DS3231_SetTime(&rtc);
	HAL_Delay(500);
	/*
	for(int i = 0; i < 5; i++) {
lcdCommand(0x00);
HAL_Delay(10);
	}		
	lcdCommand(0x30); //4-bit mode, 2 lines
	lcdCommand(0x02); //display init
	lcdCommand(0x08);
  lcdCommand(0x17);
  lcdCommand(0x04 | 0x08);
  lcdCommand(0x0C); //display on
  lcdCommand(0x01); //clear
  lcdCommand(0x80); //set cursor
	*/
	lcdCommand(0x33);
	lcdCommand(0x32);
	lcdCommand(0x28);
	lcdCommand(0x01);
	lcdCommand(0x08 | 0x04);
	lcdCommand(0x04 | 0x02);

	//WS0010
	lcdCommand(0x08);
	lcdCommand(0x17);
	lcdCommand(0x01);
	lcdCommand(0x04 | 0x08);
	lcdCommand(0x01); //clear
  lcdCommand(0x80); //set cursor
	//lcdString("+79080665015");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		    DS3231_GetTime(&rtc);
    DS3231_ReadTemperature(&rtcTemp);
		lcdSetCursor(0,0);
		uint8_t hour = rtc.Hour;
		uint8_t second = rtc.Sec;
		uint8_t minute = rtc.Min;
		if(hour < 10) {
			lcdChar('0');
lcdInt(hour);
		}
		else lcdInt(hour);
		lcdChar(':');
		if(minute < 10) {
			lcdChar('0');
lcdInt(minute);
		}
		else lcdInt(minute);
		lcdChar(':');
		if(second < 10) {
			lcdChar('0');
lcdInt(second);
		}
		else lcdInt(second);
		lcdSetCursor(0,1);
	//	lcdString("88888");
		HAL_Delay(250);
		
		uint8_t month = rtc.Month;
		uint8_t day = rtc.Date;
		uint8_t year = rtc.Year;
		lcdString("20");
		if(year < 10) {
			lcdChar('0');
lcdInt(year);
		}
    else lcdInt(year);
		lcdChar('/');
		if(month < 10) {
			lcdChar('0');
lcdInt(month);
		}
		else lcdInt(month);
		lcdChar('/');
		if(day < 10) {
			lcdChar('0');
lcdInt(day);
		}
		else lcdInt(day);
	
    ReadRegister(DS3231_REG_STATUS, &regVal);
    if(regVal & DS3231_STA_A1F)
    {
      regVal &= ~DS3231_STA_A1F;
      WriteRegister(DS3231_REG_STATUS, regVal);
    }
		callButton();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA13 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
