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
#include "kalman_filter.h"
#include "kalman_filter_real.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  bool tagProv;
  bool tagSync;
  bool tagSfSync;
  uint8_t notSyncCnt;
  uint32_t timeSyncRecv;
  uint32_t timeSyncEnd;
  uint32_t timeOffsetStart;
  uint32_t timeOffsetEnd;
  uint16_t beaconBitmap;
  uint8_t beaconRecvCnt;
  uint8_t respRecvCnt;
  uint32_t miss[BCN_SLOT_TOTAL];
  uint32_t count;
  bool kalmanInit;
  bool kalmanUse;
  double vTag;
} systemHandle_t;

typedef struct
{
  uint32_t bcnEnd[BCN_SLOT_TOTAL];
  uint32_t pollStart;
  uint32_t pollSent;
  uint32_t pollEnd;
  uint32_t respRecv[4];
  uint32_t respEnd[4];
  uint32_t locStart;
  uint32_t locSent;
  uint32_t locEnd;
} rtls_time_t;

typedef struct
{
  float rssi;
  int bcnSlotIndex;
} rssiBuf_t;

typedef struct
{
  float x;
  float y;
  uint32_t time;
} locTime_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STOP_THRESHOLD 3.6

#define TEST
#define USE_V_THRESHOLD
#define V_THRESHOLD 72.0
#define FPATH_THRESHOLD 10
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1);
  return ch;
}

#define RX_BEACON_TIMEOUT 600
#define RX_POLL_TIMEOUT 600
#define RX_RESP_TIMEOUT 800
#define RX_LOC_TIMEOUT 600
#define RESP_DELAY 360
#define US_TO_DECAWAVE_TICK 63898
#define ANTENNA_DELAY 16475
#define SPEED_OF_LIGHT 299792458
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_1 (-1.0e6 / 3494.4e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_2 (-1.0e6 / 3993.6e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_3 (-1.0e6 / 4492.8e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_5 (-1.0e6 / 6489.6e6)
#define FREQ_OFFSET_MULTIPLIER (998.4e6 / 2.0 / 1024.0 / 131072.0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const double dwFreq = 499.2e6 * 128;
static const uint8_t tagBaseAddr[2] = {0x00, 0x01};
systemHandle_t systemHandle = {0};
tagTypes_t myTag = {0};

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
    .pulseFrequency = TX_PULSE_FREQ_16MHZ,
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

rtls_time_t rtlsTimes = {0};
locTime_t locTime[16] = {0};
uint8_t locTimeIndex = 0;
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

void getFourAnchor(beaconHandle_t *beacon, uint16_t *bcnSlotBitmap)
{
  rssiBuf_t temp;
  rssiBuf_t rssiBuf[BCN_SLOT_TOTAL] = {0};
  int index = 0;

  for (int i = 0; i < BCN_SLOT_TOTAL; i++)
  {
    if (systemHandle.beaconBitmap & (1 << i))
    {
      rssiBuf[index].rssi = beacon[i].rssi;
      rssiBuf[index].bcnSlotIndex = i;
      index++;
    }
  }
  for (int i = 0; i < index - 1; i++)
  {
    for (int j = i + 1; j < index; j++)
    {
      if (rssiBuf[i].rssi < rssiBuf[j].rssi)
      {
        memcpy(&temp, &rssiBuf[i], sizeof(rssiBuf[i]));
        memcpy(&rssiBuf[i], &rssiBuf[j], sizeof(rssiBuf[j]));
        memcpy(&rssiBuf[j], &temp, sizeof(temp));
      }
    }
  }
  *bcnSlotBitmap = 0;
  for (int i = 0; i < 4; i++)
  {
    *bcnSlotBitmap |= (1 << rssiBuf[i].bcnSlotIndex);
  }
}

int getNearAnchor(beaconHandle_t *beacon)
{
  rssiBuf_t temp;
  rssiBuf_t rssiBuf[BCN_SLOT_TOTAL] = {0};
  int index = 0;

  for (int i = 0; i < BCN_SLOT_TOTAL; i++)
  {
    if (systemHandle.beaconBitmap & (1 << i))
    {
      rssiBuf[index].rssi = beacon[i].rssi;
      rssiBuf[index].bcnSlotIndex = i;
      index++;
    }
  }
  for (int i = 0; i < index - 1; i++)
  {
    for (int j = i + 1; j < index; j++)
    {
      if (rssiBuf[i].rssi < rssiBuf[j].rssi)
      {
        memcpy(&temp, &rssiBuf[i], sizeof(rssiBuf[i]));
        memcpy(&rssiBuf[i], &rssiBuf[j], sizeof(rssiBuf[j]));
        memcpy(&rssiBuf[j], &temp, sizeof(temp));
      }
    }
  }
  return rssiBuf[0].bcnSlotIndex;
}

bool calLocation(float x1, float y1, float x2, float y2, float x3, float y3, float d1, float d2, float d3, float *x, float *y)
{
  float A, B, C, D, E, F;
  if (d1 < 30 && d2 < 30 && d3 < 30 && d1 > 0 && d2 > 0 && d3 > 0)
  {
    A = 2 * (x2 - x1);
    B = 2 * (y2 - y1);
    C = d1 * d1 - d2 * d2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2;
    D = 2 * (x3 - x1);
    E = 2 * (y3 - y1);
    F = d1 * d1 - d3 * d3 - x1 * x1 + x3 * x3 - y1 * y1 + y3 * y3;
    if (E * A - B * D != 0.0)
    {
      *x = (C * E - F * B) / (E * A - B * D);
      *y = (C * D - A * F) / (B * D - A * E);
      if (!isnan(*x) && !isnan(*y) && !isinf(*x) && !isinf(*y))
        return true;
      else
        return false;
    }
    return false;
  }
  else
    return false;
}

bool getLocation(respHandle_t *resp, float *x, float *y)
{
  if (systemHandle.respRecvCnt == 4)
  {
    bool s[4] = {0};
    int cnt = 0;
    float x_sum = 0;
    float y_sum = 0;
    float xVal[4] = {0};
    float yVal[4] = {0};
    s[0] = calLocation(resp[0].respMess.x.xVal, resp[0].respMess.y.yVal, resp[1].respMess.x.xVal, resp[1].respMess.y.yVal, resp[2].respMess.x.xVal, resp[2].respMess.y.yVal, resp[0].distance, resp[1].distance, resp[2].distance, &xVal[0], &yVal[0]);
    s[1] = calLocation(resp[0].respMess.x.xVal, resp[0].respMess.y.yVal, resp[1].respMess.x.xVal, resp[1].respMess.y.yVal, resp[3].respMess.x.xVal, resp[3].respMess.y.yVal, resp[0].distance, resp[1].distance, resp[3].distance, &xVal[1], &yVal[1]);
    s[2] = calLocation(resp[3].respMess.x.xVal, resp[3].respMess.y.yVal, resp[1].respMess.x.xVal, resp[1].respMess.y.yVal, resp[2].respMess.x.xVal, resp[2].respMess.y.yVal, resp[3].distance, resp[1].distance, resp[2].distance, &xVal[2], &yVal[2]);
    s[3] = calLocation(resp[0].respMess.x.xVal, resp[0].respMess.y.yVal, resp[2].respMess.x.xVal, resp[2].respMess.y.yVal, resp[3].respMess.x.xVal, resp[3].respMess.y.yVal, resp[0].distance, resp[2].distance, resp[3].distance, &xVal[3], &yVal[3]);
    for (int i = 0; i < 4; i++)
    {
      if (s[i])
      {
        cnt++;
        x_sum += xVal[i];
        y_sum += yVal[i];
      }
    }
    if (cnt > 0)
    {
      *x = x_sum / (float)cnt;
      *y = y_sum / (float)cnt;
      return true;
    }
    else
      return false;
  }
  else
  {
    int index[3] = {0};
    bool s = false;
    int j = 0;
    for (int i = 0; i < 4; i++)
    {
      if (resp[i].distance > 0)
        index[j++] = i;
    }
    s = calLocation(resp[index[0]].respMess.x.xVal, resp[index[0]].respMess.y.yVal, resp[index[1]].respMess.x.xVal, resp[index[1]].respMess.y.yVal, resp[index[2]].respMess.x.xVal, resp[index[2]].respMess.y.yVal, resp[index[0]].distance, resp[index[1]].distance, resp[index[2]].distance, x, y);
    if (s)
      return true;
    else
      return false;
  }
}

bool getLocationNLOS(respHandle_t *resp, float *x, float *y)
{
  int fPathOk[4] = {0};
  int fPathOkCnt = 0;
  for (int i = 0; i < systemHandle.respRecvCnt; i++)
  {
    if (resp[i].fPath < FPATH_THRESHOLD && resp[i].confidenceLevel > 0.0)
    {
      fPathOk[fPathOkCnt++] = i;
    }
//    else
//    	resp[i].distance -= 0.4;
  }
  if (fPathOkCnt < 3)
    return getLocation(resp, x, y);
  else if (fPathOkCnt == 4)
  {
    bool s[4] = {0};
    int cnt = 0;
    float x_sum = 0;
    float y_sum = 0;
    float xVal[4] = {0};
    float yVal[4] = {0};
    s[0] = calLocation(resp[0].respMess.x.xVal, resp[0].respMess.y.yVal, resp[1].respMess.x.xVal, resp[1].respMess.y.yVal, resp[2].respMess.x.xVal, resp[2].respMess.y.yVal, resp[0].distance, resp[1].distance, resp[2].distance, &xVal[0], &yVal[0]);
    s[1] = calLocation(resp[0].respMess.x.xVal, resp[0].respMess.y.yVal, resp[1].respMess.x.xVal, resp[1].respMess.y.yVal, resp[3].respMess.x.xVal, resp[3].respMess.y.yVal, resp[0].distance, resp[1].distance, resp[3].distance, &xVal[1], &yVal[1]);
    s[2] = calLocation(resp[3].respMess.x.xVal, resp[3].respMess.y.yVal, resp[1].respMess.x.xVal, resp[1].respMess.y.yVal, resp[2].respMess.x.xVal, resp[2].respMess.y.yVal, resp[3].distance, resp[1].distance, resp[2].distance, &xVal[2], &yVal[2]);
    s[3] = calLocation(resp[0].respMess.x.xVal, resp[0].respMess.y.yVal, resp[2].respMess.x.xVal, resp[2].respMess.y.yVal, resp[3].respMess.x.xVal, resp[3].respMess.y.yVal, resp[0].distance, resp[2].distance, resp[3].distance, &xVal[3], &yVal[3]);
    for (int i = 0; i < 4; i++)
    {
      if (s[i])
      {
        cnt++;
        x_sum += xVal[i];
        y_sum += yVal[i];
      }
    }
    if (cnt > 0)
    {
      *x = x_sum / (float)cnt;
      *y = y_sum / (float)cnt;
      return true;
    }
    else
      return false;
  }
  else
  {
    bool s = false;
    s = calLocation(resp[fPathOk[0]].respMess.x.xVal, resp[fPathOk[0]].respMess.y.yVal,
                    resp[fPathOk[1]].respMess.x.xVal, resp[fPathOk[1]].respMess.y.yVal,
                    resp[fPathOk[2]].respMess.x.xVal, resp[fPathOk[2]].respMess.y.yVal,
                    resp[fPathOk[0]].distance, resp[fPathOk[1]].distance, resp[fPathOk[2]].distance, x, y);
    if (s)
      return true;
    else
      return false;
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    if (locTimeIndex >= 2)
    {
      float dis = sqrt(pow(locTime[0].x - locTime[locTimeIndex - 1].x, 2) + pow(locTime[0].y - locTime[locTimeIndex - 1].y, 2));
      float t = (locTime[locTimeIndex - 1].time - locTime[0].time) / 1000.0;
      systemHandle.vTag = (dis / t) * 3.6;
      if (systemHandle.vTag < V_THRESHOLD)
        systemHandle.kalmanUse = true;
      else
      {
        systemHandle.kalmanUse = false;
        systemHandle.kalmanInit = false;
      }
    }
//    else
//    {
//      systemHandle.kalmanUse = false;
//      systemHandle.kalmanInit = false;
//    }
    memset(locTime, 0, sizeof(locTime));
    locTimeIndex = 0;
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
  beaconHandle_t beaconHandle[BCN_SLOT_TOTAL] = {0};
  pollHandle_t pollHandle = {0};
  respHandle_t respHandle[4] = {0};
  locMess_t locMess = {0};
  packet_t beaconPacket = {0};
  packet_t pollPacket = {0};
  packet_t respPacket = {0};
  packet_t locPacket = {0};
  packet_t bufPacket = {0};
  float clockOffsetRatio = 0.0;
  uint32_t bcnTimes[BCN_SLOT_TOTAL] = {0};
  uint32_t twrTimes[TWR_SLOT_TOTAL] = {0};
  kalmanFilter xFilter;
  kalmanFilter yFilter;

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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  logi("TAG START");
  for (uint32_t i = 0; i < superframe.bcnSlotTotal; i++)
  {
    bcnTimes[i] = SF_BCN_SLOT_TIME * i;
  }
  for (uint32_t j = 0; j < superframe.twrSlotTotal; j++)
  {
    twrTimes[j] = (SF_TWR_SLOT_TIME * j + SF_BCN_SLOT_TIME * superframe.bcnSlotTotal);
  }

  memcpy(myTag.tagAddress, tagBaseAddr, sizeof(tagBaseAddr));

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
  TIM5->CNT = 0;
  HAL_TIM_Base_Start(&htim5);
  TIM3->CNT = 0;
  HAL_TIM_Base_Start_IT(&htim3);
  TIM2->CNT = 0;
  HAL_TIM_Base_Start(&htim2);

  //  HAL_SuspendTick();
  //  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  //  HAL_ResumeTick();
  //  HAL_Delay(1000000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (!systemHandle.tagSync)
    {
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
                TIM2->CNT = bcnTimes[bcnSlotNum] + superframe.sfOffset + systemHandle.timeSyncEnd + 100000 - systemHandle.timeSyncRecv;
              }
              superframe.sfNumberCurrent = sfNumCurrent;
              systemHandle.tagSync = true;
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
      memset(beaconHandle, 0, sizeof(beaconHandle_t) * BCN_SLOT_TOTAL);
      systemHandle.count++;
      systemHandle.beaconRecvCnt = 0;
      systemHandle.beaconBitmap = 0;
      while (!((TIM2->CNT > 0) && (TIM2->CNT < 10)))
        ;
      superframe.bcnSlotCurrent = 0;
      systemHandle.tagSfSync = false;
      while (superframe.bcnSlotCurrent < superframe.bcnSlotTotal)
      {
        while (TIM2->CNT < bcnTimes[superframe.bcnSlotCurrent])
          ;
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
              uint8_t sfNumCurrent = beaconPacket.payload[BEAC_SF_NUM];
              if (sfNumCurrent < superframe.sfNumberTotal)
                superframe.sfNumberCurrent = sfNumCurrent;
              systemHandle.timeSyncEnd = TIM2->CNT;
              if (systemHandle.tagSfSync == false)
              {
                if (systemHandle.timeSyncEnd > systemHandle.timeSyncRecv)
                {
                  TIM2->CNT = bcnTimes[superframe.bcnSlotCurrent] + superframe.sfOffset + systemHandle.timeSyncEnd - systemHandle.timeSyncRecv;
                }
                else
                {
                  TIM2->CNT = bcnTimes[superframe.bcnSlotCurrent] + superframe.sfOffset + systemHandle.timeSyncEnd + 100000 - systemHandle.timeSyncRecv;
                }
                systemHandle.tagSfSync = true;
              }
              memcpy((uint8_t *)&beaconHandle[superframe.bcnSlotCurrent].beaconMess, beaconPacket.payload, sizeof(beaconHandle[superframe.bcnSlotCurrent].beaconMess));
#ifdef TEST
              systemHandle.tagProv = true;
              myTag.twrSlotNumber = tagBaseAddr[1];
#endif
              if (systemHandle.tagProv == false)
              {
                if (beaconHandle[superframe.bcnSlotCurrent].beaconMess.flag & (1 << PROV_TAG))
                {
                  if (!memcmp(beaconHandle[superframe.bcnSlotCurrent].beaconMess.beaconExt.tagAddr, myTag.tagAddress, sizeof(myTag.tagAddress)))
                  {
                    systemHandle.tagProv = true;
                    myTag.twrSlotNumber = beaconHandle[superframe.bcnSlotCurrent].beaconMess.beaconExt.twrSlotNumber;
                  }
                }
              }
              else
              {
                if (beaconHandle[superframe.bcnSlotCurrent].beaconMess.flag & (1 << DEL_TAG))
                {
                  if (!memcmp(beaconHandle[superframe.bcnSlotCurrent].beaconMess.beaconExt.tagAddr, myTag.tagAddress, sizeof(myTag.tagAddress)))
                  {
                    HAL_NVIC_SystemReset();
                    systemHandle.tagProv = false;
                    myTag.twrSlotNumber = 0;
                  }
                }
              }
              beaconHandle[superframe.bcnSlotCurrent].rssi = dwGetReceivePower(&device);
              memcpy(beaconHandle[superframe.bcnSlotCurrent].anchorAddr, beaconPacket.sourceAddress, sizeof(beaconPacket.sourceAddress));
              systemHandle.beaconBitmap |= (1 << superframe.bcnSlotCurrent);
              systemHandle.beaconRecvCnt++;
            }
          }
        }
        else
        {
          systemHandle.miss[superframe.bcnSlotCurrent]++;
        }
        rtlsTimes.bcnEnd[superframe.bcnSlotCurrent] = TIM2->CNT;
        superframe.bcnSlotCurrent++;
      }
      if (systemHandle.tagSfSync == true)
      {
        systemHandle.notSyncCnt = 0;
      }
      else
      {
        systemHandle.notSyncCnt++;
        if (systemHandle.notSyncCnt >= 5)
        {
          systemHandle.tagSync = false;
        }
      }
      // TWR
      if (systemHandle.tagProv == true && myTag.twrSlotNumber >= superframe.twrSlotTotal * superframe.sfNumberCurrent && myTag.twrSlotNumber < superframe.twrSlotTotal * (superframe.sfNumberCurrent + 1) && systemHandle.beaconRecvCnt >= 3)
      {
        superframe.twrSlotCurrent = 0;
        while (superframe.twrSlotCurrent < superframe.twrSlotTotal)
        {
          while (TIM2->CNT < twrTimes[superframe.twrSlotCurrent])
            ;
          if (superframe.twrSlotCurrent + superframe.twrSlotTotal * superframe.sfNumberCurrent == myTag.twrSlotNumber)
          {
            while (TIM2->CNT < twrTimes[superframe.twrSlotCurrent] + 100)
              ;
            rtlsTimes.pollStart = TIM2->CNT;
            memset(&pollHandle, 0, sizeof(pollHandle));
            memset(&pollPacket, 0, sizeof(pollPacket));
            pollHandle.pollMess.messID = TYPE_POLL;
            pollHandle.pollMess.twrSlotNumber = superframe.twrSlotCurrent;
            if (systemHandle.beaconRecvCnt > 4)
            {
              getFourAnchor(beaconHandle, &pollHandle.pollMess.bcnSlotBitmap);
            }
            else
            {
              pollHandle.pollMess.bcnSlotBitmap = systemHandle.beaconBitmap;
            }
            int index = 0;
            for (int i = 0; i < BCN_SLOT_TOTAL; i++)
            {
              if (pollHandle.pollMess.bcnSlotBitmap & (1 << i))
              {
                memcpy(&pollHandle.pollMess.anchorAddr[index * 2], beaconHandle[i].anchorAddr, sizeof(beaconHandle[i].anchorAddr));
                index++;
              }
            }
            MAC80215_PACKET_INIT(pollPacket, MAC802154_TYPE_DATA);
            memset(pollPacket.destAddress, 0xFF, sizeof(pollPacket.destAddress));
            memcpy(pollPacket.sourceAddress, myTag.tagAddress, sizeof(myTag.tagAddress));
            memcpy(pollPacket.payload, (uint8_t *)&pollHandle.pollMess, sizeof(pollHandle.pollMess));
            dwNewTransmit(&device);
            dwSetDefaults(&device);
            dwSetData(&device, (uint8_t *)&pollPacket, MAC802154_HEADER_LENGTH + sizeof(pollHandle.pollMess));
            dwStartTransmit(&device, false);
            do
            {
              dwReadSystemEventStatusRegister(&device);
            } while (!(device.sysstatus[0] & (1 << TXFRS_BIT)));
            rtlsTimes.pollSent = TIM2->CNT;
            dwInteruptHandler();
            dwGetTransmitTimestamp(&device, &pollHandle.timestamp);
            pollHandle.timestamp.timeFull += ANTENNA_DELAY;
            uint8_t respIndex = 0;
            rtlsTimes.pollEnd = TIM2->CNT;
            systemHandle.respRecvCnt = 0;
            while (respIndex < 4)
            {
              while (TIM2->CNT < twrTimes[superframe.twrSlotCurrent] + (SF_POLL_SLOT_TIME + SF_RESP_SLOT_TIME * respIndex))
                ;
              dwSetReceiveWaitTimeout(&device, RX_RESP_TIMEOUT);
              dwWriteSystemConfigurationRegister(&device);
              dwNewReceive(&device);
              dwSetDefaults(&device);
              dwStartReceive(&device);
              do
              {
                dwReadSystemEventStatusRegister(&device);
              } while (!((device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8)) || (device.sysstatus[2] & ((1 << RXRFTO_BIT) >> 16))));
              rtlsTimes.respRecv[respIndex] = TIM2->CNT;
              dwInteruptHandler();
              if (device.sysstatus[1] & (((1 << RXDFR_BIT) | (1 << RXFCG_BIT)) >> 8))
              {
                int length = dwGetDataLength(&device);
                if (length > 0)
                {
                  dwGetData(&device, (uint8_t *)&respPacket, length);
                  if (!memcmp(respPacket.destAddress, tagBaseAddr, 2) && respPacket.payload[RESP_ID] == TYPE_RESP)
                  {
                    memcpy(&respHandle[respIndex].respMess, respPacket.payload, sizeof(respHandle[respIndex].respMess));
                    dwGetReceiveTimestamp(&device, &respHandle[respIndex].timestamp);
                    respHandle[respIndex].timestamp.timeFull -= ANTENNA_DELAY;
                    clockOffsetRatio = dwReadCarrierIntegrator(&device) * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_2 / 1.0e6);
                    dwTimestamp_t timeRx, timeTx;
                    memcpy(timeRx.timeRaw, respHandle[respIndex].respMess.rxTimestamp.timeRaw, sizeof(respHandle[respIndex].respMess.rxTimestamp.timeRaw));
                    memcpy(timeTx.timeRaw, respHandle[respIndex].respMess.txTimestamp.timeRaw, sizeof(respHandle[respIndex].respMess.txTimestamp.timeRaw));
                    timeRx.timeFull -= ANTENNA_DELAY;
                    timeTx.timeFull += ANTENNA_DELAY;
                    uint32_t timeInit = respHandle[respIndex].timestamp.timeLow32 - pollHandle.timestamp.timeLow32;
                    uint32_t timeResp = timeTx.timeLow32 - timeRx.timeLow32;
                    double tof = ((double)timeInit - (double)timeResp * (1 - clockOffsetRatio)) / 2.0;
                    respHandle[respIndex].distance = (float)(tof * SPEED_OF_LIGHT / dwFreq);
                    respHandle[respIndex].fPath = dwGetFirstPathPower(&device);
                    float prNlos, mc;
                    respHandle[respIndex].confidenceLevel = dwGetConfidenceLevel(&device, &prNlos, &mc);
                    respHandle[respIndex].prNlos = prNlos;
                    respHandle[respIndex].mc = mc;
                    if (respHandle[respIndex].distance < 0)
                      respHandle[respIndex].distance = 0.001;
                    systemHandle.respRecvCnt++;
                  }
                }
              }
              rtlsTimes.respEnd[respIndex] = TIM2->CNT;
              respIndex++;
            }
            while (TIM2->CNT < twrTimes[superframe.twrSlotCurrent] + (SF_POLL_SLOT_TIME + SF_RESP_SLOT_TIME * 4))
              ;
            if (systemHandle.respRecvCnt >= 3)
            {
              while (TIM2->CNT < twrTimes[superframe.twrSlotCurrent] + ((SF_POLL_SLOT_TIME + SF_RESP_SLOT_TIME * 4)) + 100)
                ;
              memset(&locPacket, 0, sizeof(locPacket));
              memset(&locMess, 0, sizeof(locMess));
              rtlsTimes.locStart = TIM2->CNT;
              float xMeas, yMeas;
              int bcnIndex = getNearAnchor(beaconHandle);
#ifndef USE_V_THRESHOLD
              if (getLocation(respHandle, &xMeas, &yMeas) == true)
              {
                if (systemHandle.kalmanInit == false)
                {
                  kalman_init(&xFilter, 0.2, 0.1, 0.01, xMeas);
                  kalman_init(&yFilter, 0.2, 0.1, 0.01, yMeas);
                  systemHandle.kalmanInit = true;
                }
                if (!isnan(xMeas) && !isnan(yMeas) && !isinf(xMeas) && !isinf(yMeas))
                {
                  locMess.x.xVal = xMeas;
                  locMess.y.yVal = yMeas;
                  // 		              locMess.x.xVal = updateEstimate(&xFilter, xMeas);
                  //                    locMess.y.yVal = updateEstimate(&yFilter, yMeas);
                }
                else
                {
                  locMess.x.xVal = 0.0;
                  locMess.y.yVal = 0.0;
                }
              }
#else
              if (getLocation(respHandle, &xMeas, &yMeas) == true)
              {
                if (!isnan(xMeas) && !isnan(yMeas) && !isinf(xMeas) && !isinf(yMeas))
                {
                  locTime[locTimeIndex].x = xMeas;
                  locTime[locTimeIndex].y = yMeas;
                  locTime[locTimeIndex].time = HAL_GetTick();
                  locTimeIndex++;
                  systemHandle.kalmanUse = false;
                  if (systemHandle.kalmanUse == true)
                  {
                    if (systemHandle.kalmanInit == false)
                    {
                    	kalman_init(&xFilter, 0.5, 0.2, 0.01, xMeas);
						kalman_init(&yFilter, 0.5, 0.2, 0.01, yMeas);
						systemHandle.kalmanInit = true;
                    }
                    if (systemHandle.vTag > STOP_THRESHOLD)
                    {
                      locMess.x.xVal = updateEstimate(&xFilter, xMeas);
                      locMess.y.yVal = updateEstimate(&yFilter, yMeas);
                      locMess.x.xVal = updateEstimate(&xFilter, xMeas);
                      locMess.y.yVal = updateEstimate(&yFilter, yMeas);
                    }
                    else
                    {
					  locMess.x.xVal = updateEstimate(&xFilter, xMeas);
					  locMess.y.yVal = updateEstimate(&yFilter, yMeas);
					  locMess.x.xVal = updateEstimate(&xFilter, xMeas);
					  locMess.y.yVal = updateEstimate(&yFilter, yMeas);
                    }
                  }
                  else
                  {
                    locMess.x.xVal = xMeas;
                    locMess.y.yVal = yMeas;
                  }
                }
                else
                {
                  locMess.x.xVal = 0.0;
                  locMess.y.yVal = 0.0;
                }
              }
#endif
              locMess.messID = TYPE_LOC;
              locMess.twrSlotNumber = myTag.twrSlotNumber;
              MAC80215_PACKET_INIT(locPacket, MAC802154_TYPE_DATA);
              memcpy(locPacket.destAddress, beaconHandle[bcnIndex].anchorAddr, sizeof(beaconHandle[bcnIndex].anchorAddr));
              memcpy(locPacket.sourceAddress, myTag.tagAddress, sizeof(locPacket.sourceAddress));
              memcpy(locPacket.payload, (uint8_t *)&locMess, sizeof(locMess));
              dwNewTransmit(&device);
              dwSetDefaults(&device);
              dwSetData(&device, (uint8_t *)&locPacket, MAC802154_HEADER_LENGTH + sizeof(locMess));
              dwStartTransmit(&device, false);
              do
              {
                dwReadSystemEventStatusRegister(&device);
              } while (!(device.sysstatus[0] & (1 << TXFRS_BIT)));
              rtlsTimes.locSent = TIM2->CNT;
              dwInteruptHandler();
              rtlsTimes.locEnd = TIM2->CNT;
            }
          }
          superframe.twrSlotCurrent++;
        }
      }
      //      memset(&rtlsTimes, 0, sizeof(rtlsTimes));
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
