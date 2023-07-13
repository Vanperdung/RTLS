#ifndef _RTLS_TYPES_H_
#define _RTLS_TYPES_H

#define BCN_SLOT_TOTAL 16
#define TWR_SLOT_TOTAL 10
#define SF_NUMBER_DEFAULT 2

typedef enum
{
	NONE = 0,
	PROV_TAG,
	DEL_TAG,
} bcnFlag_t;

typedef struct
{
	uint8_t anchorAddress[2];
	uint8_t bcnSlotNumber;
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
} anchorTypes_t;

typedef struct 
{
	bool anchorMain;
	bcnFlag_t bcnFlag;
	uint8_t extCnt;
	uint8_t anchorAddress[2];
	uint8_t bcnSlotNumber;
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
} myAnchorTypes_t;


typedef struct
{
	uint8_t tagAddress[2];
	uint8_t twrSlotNumber;
} tagTypes_t;

typedef struct 
{
	uint8_t tagAddress[2];
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
} __attribute__((packed)) tagLoc_t;


typedef struct
{
	uint32_t sfOffset;
	uint8_t bcnSlotTotal;
	uint8_t twrSlotTotal;
	uint8_t sfNumberTotal;
	uint8_t bcnSlotCurrent;
	uint8_t twrSlotCurrent;
	uint8_t sfNumberCurrent;
} sfInfo_t;

#endif
