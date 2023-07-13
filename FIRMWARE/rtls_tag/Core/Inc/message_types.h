/*
 * message_types.h
 *
 *  Created on: Apr 14, 2023
 *      Author: dung
 */

#ifndef INC_MESSAGE_TYPES_H_
#define INC_MESSAGE_TYPES_H_
#include <stdint.h>

#define SF_BCN_SLOT_TIME 1200
#define SF_TWR_SLOT_TIME 7400
#define SF_POLL_SLOT_TIME 1200
#define SF_RESP_SLOT_TIME 1300
#define SF_LOC_SLOT_TIME 1000

#define TYPE_BEAC 0x10
#define TYPE_POLL 0x21
#define TYPE_RESP 0x32
#define TYPE_LOC 0x43

#define BEAC_ID 0
#define BEAC_FLAG 1
#define BEAC_SF_NUM 2
#define BEAC_BCN_SLOT_NUM 3
#define BEAC_EXT_TAG_ADDR 4
#define BEAC_TWR_SLOT_NUM 6

#define POLL_ID 0
#define POLL_TWR_SLOT_NUM 1
#define POLL_BCN_SLOT_BITMAP 3
#define POLL_ANCHOR_ADDR 5

#define RESP_ID 0
#define RESP_X 1
#define RESP_Y 5
#define RESP_Z 9
#define RESP_RX_TIMESTAMP 13
#define RESP_TX_TIMESTAMP 18

#define LOC_ID 0
#define LOC_TWR_SLOT_NUM 1
#define LOC_X 2
#define LOC_Y 6
#define LOC_Z 10

typedef struct
{
	uint8_t messID;
	uint8_t flag;
	uint8_t sfNumber;
	uint8_t bcnSlotNumber;
	struct
	{
		uint8_t tagAddr[2];
		uint8_t twrSlotNumber;
	} beaconExt;
} __attribute__((packed)) beaconMess_t;

typedef struct
{
	uint8_t messID;
	uint16_t twrSlotNumber;
	uint16_t bcnSlotBitmap;
	uint8_t anchorAddr[8];
} __attribute__((packed)) pollMess_t;

typedef struct
{
	uint8_t messID;
	union
	{
		float xVal;
		uint8_t xByte[4];
	} x;
	union
	{
		float yVal;
		uint8_t yByte[4];
	} y;
	union
	{
		uint8_t timeRaw[5];
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
	} rxTimestamp;
	union
	{
		uint8_t timeRaw[5];
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
	} txTimestamp;
} __attribute__((packed)) respMess_t;

typedef struct
{
	uint8_t messID;
	uint8_t twrSlotNumber;
	union
	{
		float xVal;
		uint8_t xByte[4];
	} x;
	union
	{
		float yVal;
		uint8_t yByte[4];
	} y;
} __attribute__((packed)) locMess_t;

typedef struct
{
	beaconMess_t beaconMess;
	float rssi;
	uint8_t anchorAddr[2];
} beaconHandle_t;

typedef struct
{
	pollMess_t pollMess;
	dwTimestamp_t timestamp;
} pollHandle_t;

typedef struct
{
	respMess_t respMess;
	dwTimestamp_t timestamp;
	float distance;
	float fPath;
	float prNlos;
	float mc;
	float confidenceLevel;
} respHandle_t;

typedef struct
{
	locMess_t locMess;
	dwTimestamp_t timestamp;
} locHandle_t;

#endif /* INC_MESSAGE_TYPES_H_ */
