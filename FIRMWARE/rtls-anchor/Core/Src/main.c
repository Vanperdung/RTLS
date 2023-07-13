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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "decawave/dw_api.h"
#include "log/log.h"
#include "mac/mac.h"
#include "message_types.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1);
  return ch;
}
#define TWR_SLOT_NUM 5
#define ANTENNA_DELAY 16475
#define BCN_SLOT_NUM 0
#define OFFSET 245
#define RSP_DELAY 360
#define US_TO_DECAWAVE_TICK 63898
#define X 0.3
#define Y 0.5
uint8_t anchorBaseAddr[2] = {0x89, 0x01};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//dwDeviceTypes_t device = {
//  .extendedFrameLength = FRAME_LENGTH_NORMAL,
//  .pacSize = PAC_SIZE_8,
//  .pulseFrequency = TX_PULSE_FREQ_64MHZ,
//  .dataRate = TRX_RATE_6800KBPS,
//  .preambleLength = TX_PREAMBLE_LEN_128,
//  .preambleCode = PREAMBLE_CODE_64MHZ_9,
//  .channel = CHANNEL_5,
//  .smartPower = true,
//  .frameCheck = true,
//  .permanentReceive = false,
//  .deviceMode = IDLE_MODE,
//  .forceTxPower = false,
//};

dwDeviceTypes_t device = {
  .extendedFrameLength = FRAME_LENGTH_NORMAL,
  .pacSize = PAC_SIZE_8,
  .pulseFrequency = TX_PULSE_FREQ_64MHZ,
  .dataRate = TRX_RATE_6800KBPS,
  .preambleLength = TX_PREAMBLE_LEN_128,
  .preambleCode = PREAMBLE_CODE_64MHZ_9,
  .channel = CHANNEL_5,
  .smartPower = true,
  .frameCheck = true,
  .permanentReceive = false,
  .deviceMode = IDLE_MODE,
  .forceTxPower = false,
};


typedef struct
{
	bool anchorSync;
	uint16_t timePollStart;
	uint16_t timePollEnd;
	uint16_t timeRespStart;
	uint16_t timeRespEnd;
	uint16_t timeLocStart;
	uint16_t timeLocEnd;
} systemHandle_t;
systemHandle_t systemHandle = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//
//	if (GPIO_Pin == GPIO_PIN_0)
//	{
//		systemHandle.dwIsIntr = true;
//		HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
//	}
//}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Instance == TIM2)
//	{
//		systemHandle.timeoutIsIntr = true;
//	}
//}

void dwInteruptHandler(void)
{
	dwReadSystemEventStatusRegister(&device);
	if (dwIsTransmitDone(&device))
	{
		dwClearTransmitStatus(&device);
	}
	if (dwIsReceiveTimestampAvailable(&device))
	{
		dwClearReceiveTimestampAvailableStatus(&device);
	}
	if (dwIsReceiveFailed(&device))
	{
		dwClearReceiveStatus(&device);
		dwRxSoftReset(&device);
	}
	if (dwIsReceiveTimeout(&device))
	{
		dwClearReceiveStatus(&device);
		dwRxSoftReset(&device);
	}
	if (dwIsReceiveDone(&device))
	{
		dwClearReceiveStatus(&device);
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
	pollHandle_t pollHandle;
	respHandle_t respHandle;
	locMess_t locMess;
	packet_t pollPacket;
	packet_t respPacket;
	packet_t locPacket;
	uint16_t twrTimeStart = TWR_SLOT_NUM * 4500 + 10000;
	uint16_t timePollRecv, timePollProcess, cntSync;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  logi("[ANCHOR START]");
  MAC80215_PACKET_INIT(pollPacket, MAC802154_TYPE_DATA);
  MAC80215_PACKET_INIT(respPacket, MAC802154_TYPE_DATA);
  dwInit(&device);
  if (dwConfigure(&device) == DW_ERROR_OK)
  {
    dwEnableAllLeds(&device);
  }
  else
  {
    loge("[Configure failed]");
    while (1);
  }
  dwNewConfiguration(&device);
  dwSetDefaults(&device);
  dwCommitConfiguration(&device);
  TIM1->CNT = 0;
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (!(systemHandle.anchorSync))
	  {
		  dwNewReceive(&device);
		  dwSetDefaults(&device);
		  dwStartReceive(&device);
		  do
		  {
			  dwReadSystemEventStatusRegister(&device);
		  } while (!(device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8)));
		  dwInteruptHandler();
		  timePollRecv = TIM1->CNT;
		  memset(&pollHandle.pollMess, 0, sizeof(pollHandle.pollMess));
		  int dataLen = dwGetDataLength(&device);
		  if (dataLen > 0)
		  {
			  dwGetData(&device, (uint8_t *)&pollPacket, dataLen);
			  if (pollPacket.payload[POLL_ID] != TYPE_POLL)
			  {

			  }
			  else
			  {
				  memcpy(&pollHandle.pollMess, pollPacket.payload, sizeof(pollHandle.pollMess));
				  systemHandle.anchorSync = true;
				  timePollProcess = TIM1->CNT - timePollRecv;
				  TIM1->CNT = timePollProcess + OFFSET + twrTimeStart;
				  cntSync = TIM1->CNT;
				  logi("Sync clock, CNT: %d", cntSync);
			  }
		  }
		  else
		  {

		  }
	  }
	  else
	  {
		  while (TIM1->CNT < twrTimeStart);
		  systemHandle.timePollStart = TIM1->CNT;
//		  TIM2->CNT = 1;
//		  TIM2->ARR = 600;
//		  HAL_TIM_Base_Start_IT(&htim2);
		  dwSetReceiveWaitTimeout(&device, 600);
		  dwWriteSystemConfigurationRegister(&device);
		  dwNewReceive(&device);
		  dwSetDefaults(&device);
		  dwStartReceive(&device);
		  do
		  {
			  dwReadSystemEventStatusRegister(&device);
		  } while (!((device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8)) || (device.sysstatus[2] & ((1 << RXRFTO_BIT) >> 16))));
		  dwInteruptHandler();
		  timePollRecv = TIM1->CNT;
//		  HAL_TIM_Base_Stop_IT(&htim2);
		  memset(&pollHandle.pollMess, 0, sizeof(pollHandle.pollMess));
		  if (device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8))
		  {
//			  systemHandle.dwIsIntr = false;
			  int dataLen = dwGetDataLength(&device);
			  if (dataLen > 0)
			  {
				  dwGetData(&device, (uint8_t *)&pollPacket, dataLen);
				  if (pollPacket.payload[POLL_ID] != TYPE_POLL)
				  {
					  // Recv fail
				  }
				  else
				  {
					  // Recv Poll Message from Tag
					  memcpy(&pollHandle.pollMess, pollPacket.payload, sizeof(pollHandle.pollMess));
					  timePollProcess = TIM1->CNT - timePollRecv;
					  TIM1->CNT = timePollProcess + OFFSET + twrTimeStart;
					  cntSync = TIM1->CNT;
					  uint8_t *addrIndex = (uint8_t*)strstr(pollHandle.pollMess.anchorAddr, anchorBaseAddr);
					  if (addrIndex != NULL)
					  {
						  // Have my anchor address
						  dwGetReceiveTimestamp(&device, &pollHandle.timestamp);
						  uint8_t respIndex = (uint8_t)(addrIndex - pollHandle.pollMess.anchorAddr) / 2;
						  systemHandle.timePollEnd = TIM1->CNT;
						  while (TIM1->CNT < twrTimeStart + 750 + 750 * respIndex);
						  systemHandle.timeRespStart = TIM1->CNT;
//						  while (TIM1->CNT < twrTimeStart + 750 + 750 * respIndex + 50);
						  memcpy(respPacket.destAddress, pollPacket.sourceAddress, sizeof(pollPacket.sourceAddress));
						  memcpy(respPacket.sourceAddress, anchorBaseAddr, sizeof(anchorBaseAddr));
						  respHandle.respMess.messID = TYPE_RESP;
						  respHandle.respMess.x.xVal = X;
						  respHandle.respMess.y.yVal = Y;
						  memcpy(respHandle.respMess.rxTimestamp.timeRaw, pollHandle.timestamp.timeRaw, sizeof(pollHandle.timestamp.timeRaw));
						  uint64_t delayTx = US_TO_DECAWAVE_TICK * RSP_DELAY;
						  respHandle.timestamp.timeFull = dwSetTxDelay(&device, delayTx);
						  memcpy(respHandle.respMess.txTimestamp.timeRaw, respHandle.timestamp.timeRaw, sizeof(respHandle.timestamp.timeRaw));
						  memcpy(respPacket.payload, (uint8_t*)&respHandle.respMess, sizeof(respHandle.respMess));
						  dwNewTransmit(&device);
						  dwSetDefaults(&device);
						  dwSetData(&device, (uint8_t *)&respPacket, MAC802154_HEADER_LENGTH + sizeof(respHandle.respMess));
						  dwStartTransmit(&device, true);
//						  while (!(systemHandle.dwIsIntr == true));
//						  dwReadSystemEventMaskRegister(&device);
						  do
						  {
							  dwReadSystemEventStatusRegister(&device);
						  } while (!(device.sysstatus[0] & (1 << TXFRS_BIT)));
						  dwInteruptHandler();
						  systemHandle.timeRespEnd = TIM1->CNT;
//						  systemHandle.dwIsIntr = false;

						  // Wait for Loc slot
						  while (TIM1->CNT < twrTimeStart + 750 + 750 * 4);
						  systemHandle.timeLocStart = TIM1->CNT;
						  dwSetReceiveWaitTimeout(&device, 500);
						  dwWriteSystemConfigurationRegister(&device);
						  dwNewReceive(&device);
						  dwSetDefaults(&device);
						  dwStartReceive(&device);
						  do
						  {
							  dwReadSystemEventStatusRegister(&device);
						  } while (!((device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8)) || (device.sysstatus[2] & ((1 << RXRFTO_BIT) >> 16))));
						  if (device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8))
						  {
							  // Recv Loc message
							  int dataLen = dwGetDataLength(&device);
							  if (dataLen > 0)
							  {
								  dwGetData(&device, (uint8_t *)&locPacket, dataLen);
								  if (locPacket.payload[LOC_ID] != TYPE_LOC)
								  {

								  }
								  else
								  {
									  memcpy((uint8_t*)&locMess, locPacket.payload, sizeof(locMess));
								  }
							  }
							  else
							  {
								  // RX fail
							  }
						  }
						  else if (device.sysstatus[2] & ((1 << RXRFTO_BIT) >> 16))
						  {
							  // RX timeout
//							  loge("RX Loc message timeout");
						  }
						  systemHandle.timeLocEnd = TIM1->CNT;
					  }
					  else
					  {
						  // Not find anchor address
					  }
				  }
			  }
			  else
			  {
				  // RX fail
			  }

		  }
		  else if (device.sysstatus[2] & ((1 << RXRFTO_BIT) >> 16))
		  {
			  // RX timeout
//			  dwIdle(&device);
//			  systemHandle.timeoutIsIntr = false;
//			  loge("RX Poll message timeout");
			  systemHandle.timePollEnd = TIM1->CNT;
		  }
	  }
	  printf("Poll: start %d, end %d, delta %d\r\n", systemHandle.timePollStart, systemHandle.timePollEnd, systemHandle.timePollEnd - systemHandle.timePollStart);
	  printf("Resp: start %d, end %d, delta %d\r\n", systemHandle.timeRespStart, systemHandle.timeRespEnd, systemHandle.timeRespEnd - systemHandle.timeRespStart);
	  if (locMess.messID)
	  {
		  printf("Loc: start %d, end %d, delta %d\r\n", systemHandle.timeLocStart, systemHandle.timeLocEnd, systemHandle.timeLocStart - systemHandle.timeLocStart);
		  printf("X: %0.2f, Y: %0.2f\r\n", locMess.x.xVal, locMess.y.yVal);
	  }
	  memset(&locMess, 0, sizeof(locMess));
	  while (!((TIM1->CNT > 0) && (TIM1->CNT < 10)));
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
