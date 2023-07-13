#ifndef _COMMON_USER_H_
#define _COMMON_USER_H_

#include "header.h"
typedef enum
{
    NORMAL = 0,
    SMARTCONFIG,
} gateway_mode_t;

typedef struct
{
    uint8_t address[2];
    uint8_t slot;
    union
    {
        uint8_t xStr[4];
        float xVal;
    } x;
    union
    {
        uint8_t yStr[4];
        float yVal;
    } y;
} __attribute__((packed)) cfg_anchor_t;

typedef struct 
{
    uint8_t address[2];
    uint8_t slot;
} __attribute__((packed)) packet_t;

typedef struct 
{
    uint8_t address[2];
    uint8_t sf_num;
} cfg_superframe_t;

typedef struct
{
    uint8_t bcnSlotTotal;
    uint8_t twrSlotTotal;
    uint8_t sfNumberTotal;
    cfg_anchor_t anchorInfo;
} __attribute__((packed)) info_t;

typedef packet_t cfg_tag_t;
typedef packet_t del_anchor_t;
typedef packet_t del_tag_t;
typedef uint8_t cmd_start_t;
typedef uint8_t cmd_stop_t;

typedef struct 
{
    uint16_t id;
    uint8_t method[20];
    float x;
    float y;
    uint16_t address;
    uint8_t slot;
    uint8_t sf_num;
    uint8_t url[128];
} mqtt_obj_t;

typedef struct 
{
    cfg_anchor_t cfg_anchor;
    del_anchor_t del_anchor;
    cfg_tag_t cfg_tag;
    del_tag_t del_tag;
    cfg_superframe_t cfg_sf;
    info_t info;
    cmd_start_t start_address[2];
    cmd_stop_t stop_address[2];
} rtls_obj_t;

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

esp_err_t mqtt_parse_data(char *mqtt_data, mqtt_obj_t *mqtt_obj);
#endif