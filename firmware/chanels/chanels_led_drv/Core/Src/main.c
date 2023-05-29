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
#define RXBUFFERSIZE                      TXBUFFERSIZE
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO uint32_t     Transfer_Direction = 0;
__IO uint32_t     Xfer_Complete = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Buffer used for transmission */
uint8_t aTxBuffer[4];

/* Buffer used for reception */
uint8_t aRxBuffer[4];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rxEnd = 1;
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
	  aRxBuffer[0]=0x00;
	  aRxBuffer[1]=0x00;
	  aRxBuffer[2]=0x00;
	  aRxBuffer[3]=0x00;
	  aTxBuffer[0]=0xAA;
	  aTxBuffer[1]=0xBB;
	  aTxBuffer[2]=0xCC;
	  aTxBuffer[3]=0xDD;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

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
	/* Toggle LED4: Transfer in transmission process is correct */

	Xfer_Complete = 1;
	aTxBuffer[0]++;
	aTxBuffer[1]++;
	aTxBuffer[2]++;
	aTxBuffer[3]++;

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
	/* Toggle LED4: Transfer in reception process is correct */

	Xfer_Complete = 1;
	aRxBuffer[0]=0x00;
	aRxBuffer[1]=0x00;
	aRxBuffer[2]=0x00;
	aRxBuffer[3]=0x00;
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
