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
#include "dma.h"
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
#include "rtls_types.h"
#include "uart_types.h"
#include "flash_storage.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  bool anchorCfg;
  bool anchorSync;
  bool rxBufAvai;
  bool anchorSentBcn;
  uint16_t rxBufSize;
  bool anchorSfSync;
  uint8_t notSyncCnt;
  uint32_t timeSyncRecv;
  uint32_t timeSyncEnd;
  uint32_t timeOffsetStart;
  uint32_t timeOffsetEnd;
  uint8_t locCnt;
  bool kalmanInit;
  double ratio;
  uint32_t testOneCnt;
} systemHandle_t;

typedef struct
{
  uint32_t bcnStart;
  uint32_t bcnSent;
  uint32_t bcnEnd;
  uint32_t pollRecv[TWR_SLOT_TOTAL];
  uint32_t pollEnd[TWR_SLOT_TOTAL];
  uint32_t respStart[TWR_SLOT_TOTAL];
  uint32_t respSent[TWR_SLOT_TOTAL];
  uint32_t respEnd[TWR_SLOT_TOTAL];
  uint32_t locRecv[TWR_SLOT_TOTAL];
  uint32_t locEnd[TWR_SLOT_TOTAL];
  uint32_t uartStart;
  uint32_t uartEnd;
} rtlsTimes_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1);
  return ch;
}
//#define TEST
#define DIV 1
#define RX_BEACON_TIMEOUT 600
#define RX_POLL_TIMEOUT 600
#define RX_LOC_TIMEOUT 600
#define RESP_DELAY 360
#define US_TO_DECAWAVE_TICK 63898
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifndef TEST
uint8_t anchorBaseAddr[2];
#else
uint8_t anchorBaseAddr[2] = {0x89, 0x03};
#define X 0.0
#define Y 8.43
#endif
systemHandle_t systemHandle = {0};
myAnchorTypes_t myAnchor = {0};
rtlsTimes_t rtlsTimes = {0};

sfInfo_t superframe = {
    .bcnSlotTotal = BCN_SLOT_TOTAL,
    .twrSlotTotal = TWR_SLOT_TOTAL,
    .sfNumberTotal = SF_NUMBER_DEFAULT,
    .sfOffset = 385,
    .bcnSlotCurrent = 0,
    .sfNumberCurrent = 0,
    .twrSlotCurrent = 0,
};

dwDeviceTypes_t device = {
    .extendedFrameLength = FRAME_LENGTH_NORMAL,
    .pacSize = PAC_SIZE_8,
    .pulseFrequency = TX_PULSE_FREQ_64MHZ,
    .dataRate = TRX_RATE_6800KBPS,
    .preambleLength = TX_PREAMBLE_LEN_128,
    .preambleCode = PREAMBLE_CODE_16MHZ_4,
    .channel = CHANNEL_2,
    .smartPower = true,
    .frameCheck = true,
    .permanentReceive = false,
    .deviceMode = IDLE_MODE,
    .forceTxPower = false,
};

uint8_t rxBuffer[512] = {0};
uint8_t rxByte = 0;
int rxIndex = 0;
rxState_t rxState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void dwInteruptHandler(void)
{
  // dwReadSystemEventStatusRegister(&device);
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

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if (rxState == WAIT_START_BYTE)
//   {
//     if (rxByte == '$')
//     {
//       rxState = WAIT_END_BYTE;
//       rxBuffer[rxIndex++] = rxByte;
//       HAL_UART_Receive_IT(&huart3, &rxByte, sizeof(rxByte));
//     }
//     else
//     {
//       memset(rxBuffer, 0, sizeof(rxBuffer));
//       rxIndex = 0;
//       HAL_UART_Receive_IT(&huart3, &rxByte, sizeof(rxByte));
//     }
//   }
//   else
//   {
//     rxBuffer[rxIndex++] = rxByte;
//     if (rxByte == '*')
//     {
//       rxState = WAIT_START_BYTE;
//       systemHandle.rxBufAvai = true;
//     }
//     else
//     {
//       HAL_UART_Receive_IT(&huart3, &rxByte, sizeof(rxByte));
//     }
//   }
// }

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuffer, sizeof(rxBuffer));
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3)
  {
    systemHandle.rxBufAvai = true;
    systemHandle.rxBufSize = Size;
    HAL_UART_AbortReceive_IT(&huart3);
  }
}

int findAddress(uint8_t *str, uint8_t *sub, int n)
{
  for (int i = 0; i < n; i += 2)
  {
    if (str[i] == sub[0] && str[i + 1] == sub[1])
      return i;
  }
  return -1;
}

int formatCmd(char *info, char *header, char *data, int size)
{
  int k = strlen(header);
  for (int i = 0; i < k; i++)
  {
    info[i] = header[i];
  }
  for (int j = k; j < k + size; j++)
  {
    info[j] = data[j - k];
  }
  info[k + size] = '*';
  return k + size + 1;
}

int formatLocCmd(char *info, char *header, char *data, uint8_t cnt , int size)
{
  int k = strlen(header);
  for (int i = 0; i < k; i++)
  {
    info[i] = header[i];
  }
  for (int j = k; j < k + size; j++)
  {
    info[j] = data[j - k];
  }
  info[k + size] = cnt;
  info[k + size + 1] = '*';
  return k + size + 2;
}

bool compareString(char *src1, char *src2, int len)
{
  for (int i = 0; i < len; i++)
  {
    if (src1[i] != src2[i])
      return false;
  }
  return true;
}

int max(int a, int b)
{
	return (a > b) ? a : b;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  beaconHandle_t beaconHandle = {0};
  pollHandle_t pollHandle = {0};
  respHandle_t respHandle = {0};
  locMess_t locMess = {0};
  packet_t beaconPacket = {0};
  packet_t pollPacket = {0};
  packet_t respPacket = {0};
  packet_t locPacket = {0};
  packet_t bufPacket = {0};

  uint32_t bcnTimes[BCN_SLOT_TOTAL] = {0};
  uint32_t twrTimes[TWR_SLOT_TOTAL] = {0};
  uint32_t uartTime = 0;

  rtls_obj_t rtls_obj = {0};
  tagLoc_t tagLoc[TWR_SLOT_TOTAL];
  char all[] = {0xff, 0xff};
  char null[] = {0x00, 0x00};
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
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  logi("ANCHOR START");
  dwInit(&device);
  if (dwConfigure(&device) == DW_ERROR_OK)
  {
    dwEnableAllLeds(&device);
  }
  else
  {
    loge("Configure failed");
    while (1)
      ;
  }
  dwNewConfiguration(&device);
  dwSetDefaults(&device);
  dwCommitConfiguration(&device);

  for (uint32_t i = 0; i < superframe.bcnSlotTotal; i++)
  {
    bcnTimes[i] = SF_BCN_SLOT_TIME * i / DIV;
  }
  for (uint32_t j = 0; j < superframe.twrSlotTotal; j++)
  {
    twrTimes[j] = (SF_TWR_SLOT_TIME * j + SF_BCN_SLOT_TIME * superframe.bcnSlotTotal) / DIV;
  }
  uartTime = (SF_TWR_SLOT_TIME * superframe.twrSlotTotal + SF_BCN_SLOT_TIME * superframe.bcnSlotTotal) / DIV;

#ifndef TEST
  flashReadData(ADDR_FLASH_SECTOR_7, anchorBaseAddr, sizeof(anchorBaseAddr));
  Logi("Read ID: %02x%02x", anchorBaseAddr[0], anchorBaseAddr[1]);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuffer, sizeof(rxBuffer));
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  while (!(systemHandle.anchorCfg == true))
  {
    if (systemHandle.rxBufAvai == true)
    {
      memset(&rtls_obj, 0, sizeof(rtls_obj));
      if (strstr(rxBuffer, "id") != NULL)
      {
        Logi("ID: %02x%02x", rxBuffer[3], rxBuffer[4]);
        if (!compareString(anchorBaseAddr, &rxBuffer[3], 2))
        {
          memcpy(anchorBaseAddr, &rxBuffer[3], sizeof(anchorBaseAddr));
          flashWriteData(ADDR_FLASH_SECTOR_7, anchorBaseAddr, sizeof(anchorBaseAddr));
        }
        HAL_UART_Transmit(&huart3, (uint8_t *)rxBuffer, systemHandle.rxBufSize, 100);
      }
      if (strstr(rxBuffer, "cfg_anchor") != NULL)
      {
        Logi("cfg_anchor");
        memcpy((uint8_t *)&rtls_obj.cfg_anchor, &rxBuffer[11], sizeof(rtls_obj.cfg_anchor));
        if (compareString(rtls_obj.cfg_anchor.address, anchorBaseAddr, sizeof(anchorBaseAddr)) && rtls_obj.cfg_anchor.slot < superframe.bcnSlotTotal)
        {
          memcpy(myAnchor.anchorAddress, anchorBaseAddr, sizeof(anchorBaseAddr));
          myAnchor.bcnSlotNumber = rtls_obj.cfg_anchor.slot;
          myAnchor.x.xVal = rtls_obj.cfg_anchor.x.xVal;
          myAnchor.y.yVal = rtls_obj.cfg_anchor.y.yVal;
          myAnchor.anchorCfg = true;
          if (myAnchor.bcnSlotNumber == 0)
          {
            myAnchor.anchorMain = true;
          }
          HAL_UART_Transmit(&huart3, (uint8_t *)rxBuffer, systemHandle.rxBufSize, 100);
        }
      }
      else if (strstr(rxBuffer, "del_anchor") != NULL)
      {
        Logi("del_anchor");
        memcpy((uint8_t *)&rtls_obj.del_anchor, &rxBuffer[11], sizeof(rtls_obj.del_anchor));
        if ((compareString(rtls_obj.del_anchor.address, myAnchor.anchorAddress, sizeof(myAnchor.anchorAddress)) && rtls_obj.del_anchor.slot == myAnchor.bcnSlotNumber) || compareString(rtls_obj.del_anchor.address, all, sizeof(all)))
        {
          memset(&myAnchor, 0, sizeof(myAnchor));
          HAL_UART_Transmit(&huart3, (uint8_t *)rxBuffer, systemHandle.rxBufSize, 100);
        }
      }
      else if (strstr(rxBuffer, "cfg_superframe") != NULL)
      {
        Logi("cfg_superframe");
        memcpy((uint8_t *)&rtls_obj.cfg_sf, &rxBuffer[15], sizeof(rtls_obj.cfg_sf));
        if (compareString(rtls_obj.cfg_sf.address, anchorBaseAddr, sizeof(anchorBaseAddr)) || compareString(rtls_obj.cfg_sf.address, all, sizeof(all)))
        {
          if (rtls_obj.cfg_sf.sf_num <= 5 && rtls_obj.cfg_sf.sf_num >= 1)
          {
            superframe.sfNumberTotal = rtls_obj.cfg_sf.sf_num;
            HAL_UART_Transmit(&huart3, (uint8_t *)rxBuffer, systemHandle.rxBufSize, 100);
          }
        }
      }
      else if (strstr(rxBuffer, "start") != NULL)
      {
        Logi("start");
        if (myAnchor.anchorCfg == true)
        {
          systemHandle.anchorCfg = true;
          HAL_UART_Transmit(&huart3, (uint8_t *)rxBuffer, systemHandle.rxBufSize, 100);
        }
      }
      else if (strstr(rxBuffer, "get_info") != NULL)
      {
        Logi("get_info");
        memset(rxBuffer, 0, sizeof(rxBuffer));
        rtls_obj.info.bcnSlotTotal = superframe.bcnSlotTotal;
        rtls_obj.info.sfNumberTotal = superframe.sfNumberTotal;
        rtls_obj.info.twrSlotTotal = superframe.twrSlotTotal;
        memcpy(rtls_obj.info.anchorInfo.address, myAnchor.anchorAddress, sizeof(myAnchor.anchorAddress));
        rtls_obj.info.anchorInfo.slot = myAnchor.bcnSlotNumber;
        rtls_obj.info.anchorInfo.x.xVal = myAnchor.x.xVal;
        rtls_obj.info.anchorInfo.y.yVal = myAnchor.y.yVal;
        int len = formatCmd((char *)rxBuffer, "$get_info", (char *)&rtls_obj.info, sizeof(rtls_obj.info));
        HAL_UART_Transmit(&huart3, (uint8_t *)rxBuffer, len, 100);
      }
      memset(rxBuffer, 0, sizeof(rxBuffer));
      HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuffer, sizeof(rxBuffer));
      __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
      systemHandle.rxBufAvai = false;
    }
  }
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuffer, sizeof(rxBuffer));
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
#endif

#ifdef TEST
  myAnchor.anchorCfg = true;
  if (anchorBaseAddr[1] == 0x00)
  {
    myAnchor.anchorMain = true;
    memcpy(myAnchor.anchorAddress, anchorBaseAddr, sizeof(anchorBaseAddr));
    myAnchor.bcnSlotNumber = 0;
    myAnchor.x.xVal = X;
    myAnchor.y.yVal = Y;
  }
  else
  {
    myAnchor.anchorMain = false;
    memcpy(myAnchor.anchorAddress, anchorBaseAddr, sizeof(anchorBaseAddr));
    myAnchor.bcnSlotNumber = anchorBaseAddr[1];
    myAnchor.x.xVal = X;
    myAnchor.y.yVal = Y;
  }
#endif

  TIM2->CNT = 0;
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (myAnchor.anchorMain == true)
    {
      systemHandle.anchorSync = true;
    }
    if (!systemHandle.anchorSync)
    {
      // Sync clock
      dwNewReceive(&device);
      dwSetDefaults(&device);
      dwStartReceive(&device);
      do
      {
        dwReadSystemEventStatusRegister(&device);
      } while (!((device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8)) || (device.sysstatus[2] & ((1 << RXRFTO_BIT) >> 16))));
      if (device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8))
      {
        systemHandle.timeSyncRecv = TIM2->CNT;
        dwInteruptHandler();
        memset(&bufPacket, 0, sizeof(bufPacket));
        int length = dwGetDataLength(&device);
        if (length > 0)
        {
          dwGetData(&device, (uint8_t *)&bufPacket, length);
          if (bufPacket.payload[BEAC_ID] == TYPE_BEAC)
          {
            uint8_t bcnSlotNum = bufPacket.payload[BEAC_BCN_SLOT_NUM];
            uint8_t sfNumCurrent = bufPacket.payload[BEAC_SF_NUM];
            if (bcnSlotNum < superframe.bcnSlotTotal && sfNumCurrent < superframe.sfNumberTotal)
            {
              systemHandle.timeSyncEnd = TIM2->CNT;
              if (systemHandle.timeSyncEnd > systemHandle.timeSyncRecv)
              {
                TIM2->CNT = bcnTimes[bcnSlotNum] + superframe.sfOffset + systemHandle.timeSyncEnd - systemHandle.timeSyncRecv;
              }
              else
              {
                TIM2->CNT = bcnTimes[bcnSlotNum] + superframe.sfOffset + systemHandle.timeSyncEnd + 100000 / DIV - systemHandle.timeSyncRecv;
              }
              superframe.sfNumberCurrent = sfNumCurrent;
              systemHandle.anchorSync = true;
              systemHandle.notSyncCnt = 0;
            }
          }
        }
      }
      else
      {
        dwInteruptHandler();
      }
    }
    else
    {
      // New superframe
      while (!((TIM2->CNT > 0) && (TIM2->CNT < 10)))
        ;
      if (superframe.sfNumberCurrent == superframe.sfNumberTotal - 1)
      {
        superframe.sfNumberCurrent = 0;
      }
      else
      {
        superframe.sfNumberCurrent++;
      }
      memset(&beaconHandle, 0, sizeof(beaconHandle));
      if (myAnchor.extCnt > 0)
      {
        // Add extend part into Beacon Message
        if (myAnchor.bcnFlag == PROV_TAG)
        {
          beaconHandle.beaconMess.flag = (1 << PROV_TAG);
          memcpy(beaconHandle.beaconMess.beaconExt.tagAddr, rtls_obj.cfg_tag.address, sizeof(rtls_obj.cfg_tag.address));
          beaconHandle.beaconMess.beaconExt.twrSlotNumber = rtls_obj.cfg_tag.slot;
        }
        else if (myAnchor.bcnFlag == DEL_TAG)
        {
          beaconHandle.beaconMess.flag = (1 << DEL_TAG);
          memcpy(beaconHandle.beaconMess.beaconExt.tagAddr, rtls_obj.del_tag.address, sizeof(rtls_obj.del_tag.address));
          beaconHandle.beaconMess.beaconExt.twrSlotNumber = rtls_obj.del_tag.slot;
        }
        myAnchor.extCnt--;
        if (myAnchor.extCnt == 0)
        {
          if (myAnchor.bcnFlag == PROV_TAG)
          {
            memset(&rtls_obj.cfg_tag, 0, sizeof(rtls_obj.cfg_tag));
            myAnchor.bcnFlag = NONE;
          }
          else if (myAnchor.bcnFlag == DEL_TAG)
          {
            memset(&rtls_obj.del_tag, 0, sizeof(rtls_obj.del_tag));
            myAnchor.bcnFlag = NONE;
          }
        }
      }
      else
      {
        beaconHandle.beaconMess.flag = 0;
      }
      if (myAnchor.anchorMain)
      {
        rtlsTimes.bcnStart = TIM2->CNT;
        systemHandle.testOneCnt++;
        // Anchor Main
        while (TIM2->CNT < 100 / DIV)
          ;
        beaconHandle.beaconMess.messID = TYPE_BEAC;
        beaconHandle.beaconMess.sfNumber = superframe.sfNumberCurrent;
        beaconHandle.beaconMess.bcnSlotNumber = 0;
        memset(&beaconPacket, 0, sizeof(beaconPacket));
        MAC80215_PACKET_INIT(beaconPacket, MAC802154_TYPE_DATA);
        memset(beaconPacket.destAddress, 0xFF, 2);
        memcpy(beaconPacket.sourceAddress, myAnchor.anchorAddress, sizeof(myAnchor.anchorAddress));
        memcpy(beaconPacket.payload, (uint8_t *)&beaconHandle.beaconMess, sizeof(beaconHandle.beaconMess));
        dwNewTransmit(&device);
        dwSetDefaults(&device);
        dwSetData(&device, (uint8_t *)&beaconPacket, MAC802154_HEADER_LENGTH + sizeof(beaconHandle.beaconMess));
        dwStartTransmit(&device, false);
        do
        {
          dwReadSystemEventStatusRegister(&device);
        } while (!(device.sysstatus[0] & (1 << TXFRS_BIT)));
        rtlsTimes.bcnSent = TIM2->CNT;
        dwInteruptHandler();
        rtlsTimes.bcnEnd = TIM2->CNT;
      }
      else
      {
        superframe.bcnSlotCurrent = 0;
        systemHandle.anchorSfSync = false;
        while (superframe.bcnSlotCurrent < superframe.bcnSlotTotal)
        {
          while (TIM2->CNT < bcnTimes[superframe.bcnSlotCurrent])
            ;
          if (myAnchor.bcnSlotNumber == superframe.bcnSlotCurrent)
          {
		    rtlsTimes.bcnStart = TIM2->CNT;
            while (TIM2->CNT < (bcnTimes[superframe.bcnSlotCurrent] + 100 / DIV))
              ;
            beaconHandle.beaconMess.messID = TYPE_BEAC;
            beaconHandle.beaconMess.sfNumber = superframe.sfNumberCurrent;
            beaconHandle.beaconMess.bcnSlotNumber = superframe.bcnSlotCurrent;
            memset(&beaconPacket, 0, sizeof(beaconPacket));
            MAC80215_PACKET_INIT(beaconPacket, MAC802154_TYPE_DATA);
            memset(beaconPacket.destAddress, 0xFF, 2);
            memcpy(beaconPacket.sourceAddress, myAnchor.anchorAddress, sizeof(myAnchor.anchorAddress));
            memcpy(beaconPacket.payload, (uint8_t *)&beaconHandle.beaconMess, sizeof(beaconHandle.beaconMess));
            dwNewTransmit(&device);
            dwSetDefaults(&device);
            dwSetData(&device, (uint8_t *)&beaconPacket, MAC802154_HEADER_LENGTH + sizeof(beaconHandle.beaconMess));
            dwStartTransmit(&device, false);
            do
            {
              dwReadSystemEventStatusRegister(&device);
            } while (!(device.sysstatus[0] & (1 << TXFRS_BIT)));
            rtlsTimes.bcnSent = TIM2->CNT;
            dwInteruptHandler();
            rtlsTimes.bcnEnd = TIM2->CNT;
          }
          else if (systemHandle.anchorSfSync == false)
          {
            dwSetReceiveWaitTimeout(&device, RX_BEACON_TIMEOUT);
            dwWriteSystemConfigurationRegister(&device);
            dwNewReceive(&device);
            dwSetDefaults(&device);
            dwStartReceive(&device);
            do
            {
              dwReadSystemEventStatusRegister(&device);
            } while (!((device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8)) || (device.sysstatus[2] & ((1 << RXRFTO_BIT) >> 16))));
            systemHandle.timeSyncRecv = TIM2->CNT;
            dwInteruptHandler();
            if (device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8))
            {
              int length = dwGetDataLength(&device);
              if (length > 0)
              {
                memset(&beaconPacket, 0, sizeof(beaconPacket));
                dwGetData(&device, (uint8_t *)&beaconPacket, length);
                if (beaconPacket.payload[BEAC_ID] == TYPE_BEAC)
                {
                  //				  double error = (double)systemHandle.timeSyncRecv - (double)superframe.sfOffset;
                  //				  systemHandle.ratio *= 1.0 + error / 100000.0;
                  //				  uint32_t arr = TIM2->ARR;
                  //				  arr *= systemHandle.ratio;
                  //				  TIM2->ARR = arr;
                  //				  for (uint32_t i = 0; i < superframe.bcnSlotTotal; i++)
                  //				  {
                  //				    bcnTimes[i] *= systemHandle.ratio;
                  //				  }
                  //				  for (uint32_t j = 0; j < superframe.twrSlotTotal; j++)
                  //				  {
                  //				    twrTimes[j] *= systemHandle.ratio;
                  //				  }
                  //				  uartTime *= systemHandle.ratio;
                  systemHandle.timeSyncEnd = TIM2->CNT;
                  if (systemHandle.anchorSfSync == false)
                  {
                	systemHandle.testOneCnt++;
                    if (systemHandle.timeSyncEnd > systemHandle.timeSyncRecv)
                    {
                      TIM2->CNT = bcnTimes[beaconPacket.payload[BEAC_BCN_SLOT_NUM]] + superframe.sfOffset + (systemHandle.timeSyncEnd - systemHandle.timeSyncRecv);
                    }
                    else
                    {
                      TIM2->CNT = bcnTimes[beaconPacket.payload[BEAC_BCN_SLOT_NUM]] + superframe.sfOffset + (systemHandle.timeSyncEnd + 100000 / DIV - systemHandle.timeSyncRecv);
                    }
                    systemHandle.anchorSfSync = true;
                  }
                  // memset(&beaconHandle, 0, sizeof(beaconHandle));
                  // memcpy(&beaconHandle.beaconMess, beaconPacket.payload, sizeof(beaconHandle.beaconMess));
                }
              }
            }
            else
            {
            }
          }
          superframe.bcnSlotCurrent++;
        }
        if (systemHandle.anchorSfSync == true)
        {
          systemHandle.notSyncCnt = 0;
        }
        else
        {
          systemHandle.notSyncCnt++;
          if (systemHandle.notSyncCnt >= 5)
          {
            systemHandle.anchorSync = false;
          }
        }
      }
      // TWR
      superframe.twrSlotCurrent = 0;
      systemHandle.locCnt = 0;
      memset(tagLoc, 0, superframe.twrSlotTotal * sizeof(tagLoc_t));
      while (superframe.twrSlotCurrent < superframe.twrSlotTotal)
      {
        while (TIM2->CNT < twrTimes[superframe.twrSlotCurrent])
          ;
        dwSetReceiveWaitTimeout(&device, RX_POLL_TIMEOUT);
        dwWriteSystemConfigurationRegister(&device);
        dwNewReceive(&device);
        dwSetDefaults(&device);
        dwStartReceive(&device);
        do
        {
          dwReadSystemEventStatusRegister(&device);
        } while (!((device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8)) || (device.sysstatus[2] & ((1 << RXRFTO_BIT) >> 16))));
        rtlsTimes.pollRecv[superframe.twrSlotCurrent] = TIM2->CNT;
        dwInteruptHandler();
        if (device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8))
        {
          int length = dwGetDataLength(&device);
          if (length > 0)
          {
            memset(&pollPacket, 0, sizeof(pollPacket));
            dwGetData(&device, (uint8_t *)&pollPacket, length);
            if (pollPacket.payload[POLL_ID] == TYPE_POLL)
            {
              memset(&pollHandle, 0, sizeof(pollHandle));
              memcpy(&pollHandle.pollMess, pollPacket.payload, sizeof(pollHandle.pollMess));
              int myAddrIndex = findAddress(pollHandle.pollMess.anchorAddr, myAnchor.anchorAddress, sizeof(pollHandle.pollMess.anchorAddr));
              if (myAddrIndex != -1)
              {
                // Have my address
                dwGetReceiveTimestamp(&device, &pollHandle.timestamp);
                int myRespIndex = myAddrIndex / 2;
                rtlsTimes.pollEnd[superframe.twrSlotCurrent] = TIM2->CNT;
                while (TIM2->CNT < twrTimes[superframe.twrSlotCurrent] + (SF_POLL_SLOT_TIME + myRespIndex * SF_RESP_SLOT_TIME + 100) / DIV)
                  ;
                rtlsTimes.respStart[superframe.twrSlotCurrent] = TIM2->CNT;
                memset(&respPacket, 0, sizeof(respPacket));
                memset(&respHandle, 0, sizeof(respHandle));
                MAC80215_PACKET_INIT(respPacket, MAC802154_TYPE_DATA);
                memcpy(respPacket.destAddress, pollPacket.sourceAddress, sizeof(pollPacket.sourceAddress));
                memcpy(respPacket.sourceAddress, myAnchor.anchorAddress, sizeof(myAnchor.anchorAddress));
                respHandle.respMess.messID = TYPE_RESP;
                respHandle.respMess.x.xVal = myAnchor.x.xVal;
                respHandle.respMess.y.yVal = myAnchor.y.yVal;
                memcpy(respHandle.respMess.rxTimestamp.timeRaw, pollHandle.timestamp.timeRaw, sizeof(pollHandle.timestamp.timeRaw));
                uint64_t delayTx = US_TO_DECAWAVE_TICK * RESP_DELAY;
                respHandle.timestamp.timeFull = dwSetTxDelay(&device, delayTx);
                memcpy(respHandle.respMess.txTimestamp.timeRaw, respHandle.timestamp.timeRaw, sizeof(respHandle.timestamp.timeRaw));
                memcpy(respPacket.payload, (uint8_t *)&respHandle.respMess, sizeof(respHandle.respMess));
                dwNewTransmit(&device);
                dwSetDefaults(&device);
                dwSetData(&device, (uint8_t *)&respPacket, MAC802154_HEADER_LENGTH + sizeof(respHandle.respMess));
                dwStartTransmit(&device, true);
                do
                {
                  dwReadSystemEventStatusRegister(&device);
                } while (!(device.sysstatus[0] & (1 << TXFRS_BIT)));
                rtlsTimes.respSent[superframe.twrSlotCurrent] = TIM2->CNT;
                dwInteruptHandler();
                rtlsTimes.respEnd[superframe.twrSlotCurrent] = TIM2->CNT;
                while (TIM2->CNT < twrTimes[superframe.twrSlotCurrent] + (SF_POLL_SLOT_TIME + 4 * SF_RESP_SLOT_TIME) / DIV)
                  ;
                dwSetReceiveWaitTimeout(&device, RX_LOC_TIMEOUT);
                dwWriteSystemConfigurationRegister(&device);
                dwNewReceive(&device);
                dwSetDefaults(&device);
                dwStartReceive(&device);
                do
                {
                  dwReadSystemEventStatusRegister(&device);
                } while (!((device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8)) || (device.sysstatus[2] & ((1 << RXRFTO_BIT) >> 16))));
                rtlsTimes.locRecv[superframe.twrSlotCurrent] = TIM2->CNT;
                dwInteruptHandler();
                if (device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8))
                {
                  int length = dwGetDataLength(&device);
                  if (length > 0)
                  {
                    memset(&locPacket, 0, sizeof(locPacket));
                    memset(&locMess, 0, sizeof(locMess));
                    dwGetData(&device, (uint8_t *)&locPacket, length);

                    if (locPacket.payload[LOC_ID] == TYPE_LOC && memcmp(locPacket.destAddress, myAnchor.anchorAddress, sizeof(myAnchor.anchorAddress)) == 0)
                    {
                      memcpy((uint8_t *)&locMess, locPacket.payload, sizeof(locMess));
                      memcpy(tagLoc[systemHandle.locCnt].tagAddress, locPacket.sourceAddress, sizeof(locPacket.sourceAddress));
                      tagLoc[systemHandle.locCnt].twrSlotNumber = locMess.twrSlotNumber;
                      tagLoc[systemHandle.locCnt].x.xVal = locMess.x.xVal;
                      tagLoc[systemHandle.locCnt].y.yVal = locMess.y.yVal;
                      systemHandle.locCnt++;
                    }
                  }
                  rtlsTimes.locEnd[superframe.twrSlotCurrent] = TIM2->CNT;
                }
                else
                {
                	rtlsTimes.locEnd[superframe.twrSlotCurrent] = TIM2->CNT;
                }
              }
            }
          }
        }
        else
        {
        	rtlsTimes.pollEnd[superframe.twrSlotCurrent] = TIM2->CNT;
        }
        superframe.twrSlotCurrent++;
      }
      // UART
      while (TIM2->CNT < uartTime)
        ;
      rtlsTimes.uartStart = TIM2->CNT;
      if (systemHandle.locCnt > 0)
      {
        uint8_t *tagPayload = calloc(sizeof(tagLoc_t) * systemHandle.locCnt + 20, sizeof(uint8_t));
        int len = formatLocCmd((char *)tagPayload, "$location", (char *)tagLoc, systemHandle.locCnt, sizeof(tagLoc_t) * systemHandle.locCnt);
        HAL_UART_Transmit(&huart3, tagPayload, len, 1000);
        free(tagPayload);
      }
      if (systemHandle.rxBufAvai == true)
      {
        if (strstr(rxBuffer, "stop") != NULL)
        {
          HAL_UART_Transmit(&huart3, (uint8_t *)rxBuffer, systemHandle.rxBufSize, 100);
          HAL_NVIC_SystemReset();
        }
        else if (strstr(rxBuffer, "cfg_tag") != NULL)
        {
          if (myAnchor.bcnFlag == NONE)
          {
            myAnchor.bcnFlag = PROV_TAG;
            myAnchor.extCnt = 5;
            memcpy((uint8_t *)&rtls_obj.cfg_tag, &rxBuffer[8], sizeof(rtls_obj.cfg_tag));
            HAL_UART_Transmit(&huart3, (uint8_t *)rxBuffer, systemHandle.rxBufSize, 100);
          }
        }
        else if (strstr(rxBuffer, "del_tag") != NULL)
        {
          myAnchor.bcnFlag = DEL_TAG;
          myAnchor.extCnt = 5;
          memcpy((uint8_t *)&rtls_obj.del_tag, &rxBuffer[8], sizeof(rtls_obj.del_tag));
          HAL_UART_Transmit(&huart3, (uint8_t *)rxBuffer, systemHandle.rxBufSize, 100);
        }
        else if (strstr(rxBuffer, "get_info") != NULL)
        {
          memset(rxBuffer, 0, sizeof(rxBuffer));
          rtls_obj.info.bcnSlotTotal = superframe.bcnSlotTotal;
          rtls_obj.info.sfNumberTotal = superframe.sfNumberTotal;
          rtls_obj.info.twrSlotTotal = superframe.twrSlotTotal;
          memcpy(rtls_obj.info.anchorInfo.address, myAnchor.anchorAddress, sizeof(myAnchor.anchorAddress));
          rtls_obj.info.anchorInfo.slot = myAnchor.bcnSlotNumber;
          rtls_obj.info.anchorInfo.x.xVal = myAnchor.x.xVal;
          rtls_obj.info.anchorInfo.y.yVal = myAnchor.y.yVal;
          int len = formatCmd((char *)rxBuffer, "$get_info", (char *)&rtls_obj.info, sizeof(rtls_obj.info));
          HAL_UART_Transmit(&huart3, (uint8_t *)rxBuffer, len, 100);
        }
        else if (strstr(rxBuffer, "get_test") != NULL)
        {
        	memset(rxBuffer, 0, sizeof(rxBuffer));
        	sprintf(rxBuffer, "$get_test%d*", systemHandle.testOneCnt);
        	HAL_UART_Transmit(&huart3, (uint8_t *)rxBuffer, strlen(rxBuffer), 100);
        }
        memset(rxBuffer, 0, sizeof(rxBuffer));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuffer, sizeof(rxBuffer));
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
        systemHandle.rxBufAvai = false;
      }
      rtlsTimes.uartEnd = TIM2->CNT;
    }
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

#ifdef USE_FULL_ASSERT
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
