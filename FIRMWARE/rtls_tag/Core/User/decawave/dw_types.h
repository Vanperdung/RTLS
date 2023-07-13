#ifndef _DW_TYPES_H_
#define _DW_TYPES_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "dw_regs.h"

struct dwDeviceTypes_s;
struct dwFunction_s;

typedef union
{
    uint8_t timeRaw[5];
    uint64_t timeFull;
    struct
    {
        uint32_t timeLow32;
        uint8_t timeHigh8;
    } __attribute__((packed));
    struct
    {
        uint8_t timeLow8;
        uint32_t timeHigh32;
    } __attribute__((packed));
} dwTimestamp_t;

typedef void (*dwHandler_t)(struct dwDeviceTypes_s *dev);

typedef struct dwDeviceTypes_s
{
    struct dwFunction_s *func;

    /* State */
    uint8_t sysctrl[LEN_SYS_CTRL];
    uint8_t deviceMode; // RX, TX, IDLE
    uint8_t networkAndAddress[LEN_PANADR];
    uint8_t syscfg[LEN_SYS_CFG];
    uint8_t sysmask[LEN_SYS_MASK];
    uint8_t chanctrl[LEN_CHAN_CTRL];
    uint8_t sysstatus[LEN_SYS_STATUS];
    uint8_t txfctrl[LEN_TX_FCTRL];

    uint8_t extendedFrameLength;
    uint8_t pacSize;
    uint8_t pulseFrequency;
    uint8_t dataRate;
    uint8_t preambleLength;
    uint8_t preambleCode;
    uint8_t channel;
    bool smartPower; // su dung che do SmartTX hay khong
    bool frameCheck; // lien quan den FCS(Frame check sequence), luon de true
    bool permanentReceive; // true thi chuyen sang che do RX ngay lap tuc, nen de false
    bool wait4resp; // tu dong chuyen sang RX sau khi TX xong

    dwTimestamp_t antennaDelay;

    dwHandler_t handleSent;
    dwHandler_t handleError;
    dwHandler_t handleReceived;
    dwHandler_t handleReceiveTimeout;
    dwHandler_t handleReceiveFailed;
    dwHandler_t handleReceiveTimestampAvailable;

    /* Settings */
    uint32_t txPower;
    bool forceTxPower;
} dwDeviceTypes_t;

typedef enum
{
    dwSpiSpeedLow,
    dwSpiSpeedHigh
} dwSpiSpeed_t;

typedef enum
{
    dwClockAuto = 0x00,
    dwClockXti = 0x01,
    dwClockPll = 0x02
} dwClock_t;

typedef struct dwFunction_s
{
    /**
     * Function that activates the chip-select, sends header, read data and
     * disable the chip-select.
     */
    void (*spiRead)(dwDeviceTypes_t *dev, const void *header, size_t headerLength,
                    void *data, size_t dataLength);

    /**
     * Function that activates the chip-select, sends header, sends data and
     * disable the chip-select.
     */
    void (*spiWrite)(dwDeviceTypes_t *dev, const void *header, size_t headerLength,
                     const void *data, size_t dataLength);

    /**
     * Sets the SPI bus speed. Take as argument:
     *	 - dwSpiSpeedLow: <= 4MHz
     *	 - dwSpiSpeedHigh: <= 20MHz
     */
    void (*spiSetSpeed)(dwDeviceTypes_t *dev, dwSpiSpeed_t speed);

    /**
     * Waits at least 'delay' miliseconds.
     */
    void (*delayms)(dwDeviceTypes_t *dev, unsigned int delay);

    /**
     * Resets the DW1000 by pulling the reset pin low and then releasing it.
     * This function is optional, if not set softreset via SPI will be used.
     */
    void (*reset)(dwDeviceTypes_t *dev);
} dwFunction_t;

#endif
