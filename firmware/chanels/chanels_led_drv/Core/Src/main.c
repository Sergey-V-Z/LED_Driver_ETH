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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer))
/* Size of Reception buffer */
#define RXBUFFERSIZE                      (COUNTOF(aRxBuffer))
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO uint32_t     Transfer_Direction = 0;
__IO uint32_t     Xfer_Complete = 0;
#define ln_ch 5 //chanels for adc

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim3;
/* Buffer used for transmission */
uint8_t aTxBuffer[21] = {0};

/* Buffer used for reception */
uint8_t aRxBuffer[15] = {0};

uint16_t adc_buffer[ln_ch*2] = {0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rxEnd = 1;
volatile uint32_t OwnAddr = 0;
uint8_t en1 = 0, en2 = 0, en3 = 0;
uint8_t adc_Flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

	IN_GPIO_Init();

	//Bit0
	if (LL_GPIO_IsInputPinSet(A0_PWM_CH4_GPIO_Port, A0_PWM_CH4_Pin)) {SET_BIT(OwnAddr,1<<0);}
	else {CLEAR_BIT(OwnAddr,1<<0);}
	//Bit1
	if (LL_GPIO_IsInputPinSet(A1_PWM_CH2_GPIO_Port, A1_PWM_CH2_Pin)) {SET_BIT(OwnAddr,1<<1);}
	else {CLEAR_BIT(OwnAddr,1<<1);}
	//Bit2
	if (LL_GPIO_IsInputPinSet(A2_PWM_CH1_GPIO_Port, A2_PWM_CH1_Pin)) {SET_BIT(OwnAddr,1<<2);}
	else {CLEAR_BIT(OwnAddr,1<<2);}
	//Bit3
	if (LL_GPIO_IsInputPinSet(A3_LED_GPIO_Port, A3_LED_Pin)) {SET_BIT(OwnAddr,1<<3);}
	else {CLEAR_BIT(OwnAddr,1<<3);}
	//Bit4
	SET_BIT(OwnAddr,1<<4);

	LL_GPIO_DeInit(A0_PWM_CH4_GPIO_Port);
	LL_GPIO_DeInit(A1_PWM_CH2_GPIO_Port);
	LL_GPIO_DeInit(A3_LED_GPIO_Port);


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
	{
		/* Transfer error in reception process */
		Error_Handler();
	}

	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);


	if(HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK){
		Error_Handler();
	}


	if(HAL_ADC_Start_DMA(&hadc, (uint32_t*)&adc_buffer, ln_ch) != HAL_OK){
		Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (Xfer_Complete ==1)
		{
			HAL_Delay(1);
			/*##- Put I2C peripheral in listen mode process ###########################*/
			if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
			{
				/* Transfer error in reception process */
				Error_Handler();
			}
			Xfer_Complete =0;
		}

		if(en1) LL_GPIO_SetOutputPin(EN1_GPIO_Port, EN1_Pin);
		else LL_GPIO_ResetOutputPin(EN1_GPIO_Port, EN1_Pin);

		if(en2) LL_GPIO_SetOutputPin(EN2_GPIO_Port, EN2_Pin);
		else LL_GPIO_ResetOutputPin(EN2_GPIO_Port, EN2_Pin);

		if(en3) LL_GPIO_SetOutputPin(EN3_GPIO_Port, EN3_Pin);
		else LL_GPIO_ResetOutputPin(EN3_GPIO_Port, EN3_Pin);

		if (adc_Flag){
			//обработка

			// Запретить прерывания IRQ
			__disable_irq ();

			aTxBuffer[4] = adc_buffer[0]&0xFF;
			aTxBuffer[5] = (adc_buffer[0]>>8)&0xFF;

			aTxBuffer[11] = adc_buffer[1]&0xFF;
			aTxBuffer[12] = (adc_buffer[1]>>8)&0xFF;

			aTxBuffer[18] = adc_buffer[2]&0xFF;
			aTxBuffer[19] = (adc_buffer[2]>>8)&0xFF;

			// Разрешить прерывания IRQ
			__enable_irq ();

			adc_Flag = 0;
			if(HAL_ADC_Start_DMA(&hadc, (uint32_t*)&adc_buffer, ln_ch) != HAL_OK){
				Error_Handler();
			}
		}

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {

  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Tx Transfer completed callback.
 * @param  I2cHandle: I2C handle.
 * @note   This example shows a simple way to report end of IT Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	/* Toggle LED: Transfer in transmission process is correct */
	LL_GPIO_TogglePin(A3_LED_GPIO_Port, A3_LED_Pin);

	Xfer_Complete = 1;
	/*
	aTxBuffer[0]++;
	aTxBuffer[1]++;
	aTxBuffer[2]++;
	aTxBuffer[3]++;
	 */
	aTxBuffer[0] = htim3.Instance->CCR1 & 0xFF;
	aTxBuffer[1] = (htim3.Instance->CCR1 >> 8) & 0xFF;
	aTxBuffer[2] = (htim3.Instance->CCR1 >> 16) & 0xFF;
	aTxBuffer[3] = (htim3.Instance->CCR1 >> 24) & 0xFF;
	aTxBuffer[6] = en1;

	aTxBuffer[7] = htim3.Instance->CCR2 & 0xFF;
	aTxBuffer[8] = (htim3.Instance->CCR2 >> 8) & 0xFF;
	aTxBuffer[9] = (htim3.Instance->CCR2 >> 16) & 0xFF;
	aTxBuffer[10] = (htim3.Instance->CCR2 >> 24) & 0xFF;
	aTxBuffer[13] = en2;

	aTxBuffer[14] = htim3.Instance->CCR4 & 0xFF;
	aTxBuffer[15] = (htim3.Instance->CCR4 >> 8) & 0xFF;
	aTxBuffer[16] = (htim3.Instance->CCR4 >> 16) & 0xFF;
	aTxBuffer[17] = (htim3.Instance->CCR4 >> 24) & 0xFF;
	aTxBuffer[20] = en3;

}


/**
 * @brief  Rx Transfer completed callback.
 * @param  I2cHandle: I2C handle
 * @note   This example shows a simple way to report end of IT Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	/* Toggle LED: Transfer in reception process is correct */
	LL_GPIO_TogglePin(A3_LED_GPIO_Port, A3_LED_Pin);

	Xfer_Complete = 1;
	/*
	aRxBuffer[0]=0x00;
	aRxBuffer[1]=0x00;
	aRxBuffer[2]=0x00;
	aRxBuffer[3]=0x00;
	 */

	htim3.Instance->CCR1 = aRxBuffer[3] |(aRxBuffer[2] << 8)|(aRxBuffer[1] << 16)|(aRxBuffer[0] << 24);
	en1 = aRxBuffer[4];
	htim3.Instance->CCR2 = aRxBuffer[8] |(aRxBuffer[7] << 8)|(aRxBuffer[6] << 16)|(aRxBuffer[5] << 24);
	en2 = aRxBuffer[9];
	htim3.Instance->CCR4 = aRxBuffer[13] |(aRxBuffer[12] << 8)|(aRxBuffer[11] << 16)|(aRxBuffer[10] << 24);
	en3 = aRxBuffer[14];
}



/**
 * @brief  Slave Address Match callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
 * @param  AddrMatchCode: Address Match Code
 * @retval None
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	Transfer_Direction = TransferDirection;
	if (Transfer_Direction != 0)
	{
		/*##- Start the transmission process #####################################*/
		/* While the I2C in reception process, user can transmit data through
     "aTxBuffer" buffer */
		if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t *)aTxBuffer, TXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)

		{
			/* Transfer error in transmission process */
			Error_Handler();
		}

	}
	else
	{

		/*##- Put I2C peripheral in reception process ###########################*/
		if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, (uint8_t *)aRxBuffer, RXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
		{
			/* Transfer error in reception process */
			Error_Handler();
		}

	}

}

/**
 * @brief  Listen Complete callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
}

/**
 * @brief  I2C error callbacks.
 * @param  I2cHandle: I2C handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
	/** Error_Handler() function is called when error occurs.
	 * 1- When Slave doesn't acknowledge its address, Master restarts communication.
	 * 2- When Master doesn't acknowledge the last data transferred, Slave doesn't care in this example.
	 */
	if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
	{
		Error_Handler();
	}
}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void IN_GPIO_Init(void)
{

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	/**/
	LL_GPIO_ResetOutputPin(A0_PWM_CH4_GPIO_Port, A0_PWM_CH4_Pin);

	/**/
	LL_GPIO_ResetOutputPin(A1_PWM_CH2_GPIO_Port, A1_PWM_CH2_Pin);

	/**/
	LL_GPIO_ResetOutputPin(A2_PWM_CH1_GPIO_Port, A2_PWM_CH1_Pin);

	/**/
	LL_GPIO_ResetOutputPin(A3_LED_GPIO_Port, A3_LED_Pin);

	/**/
	GPIO_InitStruct.Pin = A0_PWM_CH4_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(A0_PWM_CH4_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = A1_PWM_CH2_Pin;
	LL_GPIO_Init(A1_PWM_CH2_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = A1_PWM_CH2_Pin;
	LL_GPIO_Init(A2_PWM_CH1_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = A3_LED_Pin;
	LL_GPIO_Init(A3_LED_GPIO_Port, &GPIO_InitStruct);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	/* This is called after the conversion is completed */

	HAL_ADC_Stop_DMA(hadc);
	adc_Flag = 1;


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
