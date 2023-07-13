#ifndef _DW_API_H_
#define _DW_API_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "dw_func.h"
#include "dw_types.h"
#include "dw_spi.h"
#include "dw_regs.h"

// Default Mode of operation
extern const uint8_t MODE_LONGDATA_RANGE_LOWPOWER[];
extern const uint8_t MODE_SHORTDATA_FAST_LOWPOWER[];
extern const uint8_t MODE_LONGDATA_FAST_LOWPOWER[];
extern const uint8_t MODE_SHORTDATA_FAST_ACCURACY[];
extern const uint8_t MODE_LONGDATA_FAST_ACCURACY[];
extern const uint8_t MODE_LONGDATA_RANGE_ACCURACY[];
extern const uint8_t MODE_SHORTDATA_MID_ACCURACY[];
extern const uint8_t MODE_LONGDATA_MID_ACCURACY[];

/* Error codes */
#define DW_ERROR_OK 0
#define DW_ERROR_WRONG_ID 1

/**
 * Initialize the device data structure.
 */
void dwInit(dwDeviceTypes_t *dev);

/**
 * Setup the DW1000
 */
int dwConfigure(dwDeviceTypes_t* dev);

/**
 * Read and return the device ID, only chip with ID 0xdeca0130 is supported.
 */
uint32_t dwGetDeviceId(dwDeviceTypes_t* dev);

/**
 * Manualy blinks LEDs.
 * @param leds Bit-field of the LEDs to blink
 */
void dwEnableAllLeds(dwDeviceTypes_t* dev);

/**
 * Sets clock Mode
 */
void dwEnableClock(dwDeviceTypes_t* dev, dwClock_t clock);

/**
 * Resets the chip via SPI
 */
void dwSoftReset(dwDeviceTypes_t* dev);

void dwManageLDE(dwDeviceTypes_t* dev);

/* ###########################################################################
 * #### DW1000 register read/write ###########################################
 * ######################################################################### */

void dwReadSystemConfigurationRegister(dwDeviceTypes_t* dev);
void dwWriteSystemConfigurationRegister(dwDeviceTypes_t* dev);
void dwReadSystemEventStatusRegister(dwDeviceTypes_t* dev);
void dwReadNetworkIdAndDeviceAddress(dwDeviceTypes_t* dev);
void dwWriteNetworkIdAndDeviceAddress(dwDeviceTypes_t* dev);
void dwReadSystemEventMaskRegister(dwDeviceTypes_t* dev);
void dwWriteSystemEventMaskRegister(dwDeviceTypes_t* dev);
void dwReadChannelControlRegister(dwDeviceTypes_t* dev);
void dwWriteChannelControlRegister(dwDeviceTypes_t* dev);
void dwReadTransmitFrameControlRegister(dwDeviceTypes_t* dev);
void dwWriteTransmitFrameControlRegister(dwDeviceTypes_t* dev);

/****************************************************************/

/**
 * Set Receive Wait Timeout.
 * @param timeout Timeout in step of 1.026us (512 count of the dw1000
 *                 fundamental 499.2MHz clock) or 0 to disable the timeout.
 *
 * @note dwCommitConfiguration() should be called if this function
 * enables or disables the timeout. If the timeout is just updated and not
 * enabled this function will update to the new timeout and nothing more has to
 * be done.
 */
void dwSetReceiveWaitTimeout(dwDeviceTypes_t *dev, uint16_t timeout);

void dwSetFrameFilter(dwDeviceTypes_t* dev, bool val);
void dwSetFrameFilterBehaveCoordinator(dwDeviceTypes_t* dev, bool val);
void dwSetFrameFilterAllowBeacon(dwDeviceTypes_t* dev, bool val);
void dwSetFrameFilterAllowData(dwDeviceTypes_t* dev, bool val);
void dwSetFrameFilterAllowAcknowledgement(dwDeviceTypes_t* dev, bool val);
void dwSetFrameFilterAllowMAC(dwDeviceTypes_t* dev, bool val);
void dwSetFrameFilterAllowReserved(dwDeviceTypes_t* dev, bool val);
void dwSetDoubleBuffering(dwDeviceTypes_t* dev, bool val);
void dwSetInterruptPolarity(dwDeviceTypes_t* dev, bool val);
void dwSetReceiverAutoReenable(dwDeviceTypes_t* dev, bool val);
void dwInterruptOnSent(dwDeviceTypes_t* dev, bool val);
void dwInterruptOnReceived(dwDeviceTypes_t* dev, bool val);
void dwInterruptOnReceiveFailed(dwDeviceTypes_t* dev, bool val);
void dwInterruptOnReceiveTimeout(dwDeviceTypes_t* dev, bool val);
void dwInterruptOnReceiveTimestampAvailable(dwDeviceTypes_t* dev, bool val);
void dwInterruptOnAutomaticAcknowledgeTrigger(dwDeviceTypes_t* dev, bool val);
void dwClearInterrupts(dwDeviceTypes_t* dev);
void dwRxSoftReset(dwDeviceTypes_t *dev);


void dwIdle(dwDeviceTypes_t* dev);
void dwNewReceive(dwDeviceTypes_t* dev);
void dwStartReceive(dwDeviceTypes_t* dev);
void dwNewTransmit(dwDeviceTypes_t* dev);
void dwStartTransmit(dwDeviceTypes_t *dev, bool isTxDelay);
void dwNewConfiguration(dwDeviceTypes_t* dev);
void dwCommitConfiguration(dwDeviceTypes_t* dev);
void dwWaitForResponse(dwDeviceTypes_t* dev, bool val);
void dwSuppressFrameCheck(dwDeviceTypes_t* dev, bool val);
void dwUseSmartPower(dwDeviceTypes_t* dev, bool smartPower);
dwTimestamp_t dwSetDelay(dwDeviceTypes_t* dev, const dwTimestamp_t* delay);
uint64_t dwSetTxDelay(dwDeviceTypes_t *dev, uint64_t delay);
void dwSetTxRxTime(dwDeviceTypes_t* dev, const dwTimestamp_t futureTime);
void dwSetDataRate(dwDeviceTypes_t* dev, uint8_t rate);
void dwSetPulseFrequency(dwDeviceTypes_t* dev, uint8_t freq);
uint8_t dwGetPulseFrequency(dwDeviceTypes_t* dev);
void dwSetPreambleLength(dwDeviceTypes_t* dev, uint8_t prealen);
void dwUseExtendedFrameLength(dwDeviceTypes_t* dev, bool val);
void dwReceivePermanently(dwDeviceTypes_t* dev, bool val);
void dwSetChannel(dwDeviceTypes_t* dev, uint8_t channel);
void dwSetPreambleCode(dwDeviceTypes_t* dev, uint8_t preacode);
void dwSetDefaults(dwDeviceTypes_t* dev);
void dwSetData(dwDeviceTypes_t* dev, uint8_t data[], unsigned int n);
unsigned int dwGetDataLength(dwDeviceTypes_t* dev);
void dwGetData(dwDeviceTypes_t* dev, uint8_t data[], unsigned int n);
void dwGetTransmitTimestamp(dwDeviceTypes_t* dev, dwTimestamp_t* time);
void dwGetReceiveTimestamp(dwDeviceTypes_t* dev, dwTimestamp_t* time);
void dwGetRawReceiveTimestamp(dwDeviceTypes_t* dev, dwTimestamp_t* time);
void dwCorrectTimestamp(dwDeviceTypes_t* dev, dwTimestamp_t* timestamp);
void dwGetSystemTimestamp(dwDeviceTypes_t* dev, dwTimestamp_t* time);
bool dwIsTransmitDone(dwDeviceTypes_t* dev);
bool dwIsReceiveTimestampAvailable(dwDeviceTypes_t* dev);
bool dwIsReceiveDone(dwDeviceTypes_t* dev);
bool dwIsReceiveFailed(dwDeviceTypes_t *dev);
bool dwIsReceiveTimeout(dwDeviceTypes_t* dev);
bool dwIsClockProblem(dwDeviceTypes_t* dev);
void dwClearAllStatus(dwDeviceTypes_t* dev);
void dwClearReceiveTimestampAvailableStatus(dwDeviceTypes_t* dev);
void dwClearReceiveStatus(dwDeviceTypes_t* dev);
void dwClearTransmitStatus(dwDeviceTypes_t* dev);
float dwGetReceiveQuality(dwDeviceTypes_t* dev);
float dwGetFirstPathPower(dwDeviceTypes_t* dev);
float dwGetReceivePower(dwDeviceTypes_t* dev);
void dwEnableMode(dwDeviceTypes_t *dev, const uint8_t mode[]);
void dwTune(dwDeviceTypes_t *dev);
void dwHandleInterrupt(dwDeviceTypes_t *dev);
void dwSetSmartPower(dwDeviceTypes_t *dev);
void dwSetMode(dwDeviceTypes_t *dev);
/**
 * Set the value of the TXPower register
 */
void dwSetTxPower(dwDeviceTypes_t *dev, uint32_t txPower);

void dwAttachSentHandler(dwDeviceTypes_t *dev, dwHandler_t handler);
void dwAttachReceivedHandler(dwDeviceTypes_t *dev, dwHandler_t handler);
void dwAttachReceiveTimeoutHandler(dwDeviceTypes_t *dev, dwHandler_t handler);
void dwAttachReceiveFailedHandler(dwDeviceTypes_t *dev, dwHandler_t handler);

void dwSetAntenaDelay(dwDeviceTypes_t *dev, dwTimestamp_t delay);

/* Tune the DWM radio parameters */
void dwTune(dwDeviceTypes_t *dev);

/**
 * Put the dwm1000 in idle mode
 */
void dwIdle(dwDeviceTypes_t* dev);

/**
 * Returns a human-readable error string
 */
char* dwStrError(int error);


#endif
