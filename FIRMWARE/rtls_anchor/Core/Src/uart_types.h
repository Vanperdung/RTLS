#ifndef _UART_TYPES_H_
#define _UART_TYPES_H_

typedef enum
{
    WAIT_START_BYTE,
    WAIT_END_BYTE
} rxState_t;

/*
    Config Anchor ~ Add Anchor
    $cfgAn,[A],[B],[X],[Y],*
    [A]: Anchor address (2 byte)
    [B]: BCN slot number (1 byte)
    [X]: X location (4 byte)
    [Y]: Y location (4 byte)

    Provision Tag
    $provTag,[A],[T],*
    [A]: Tag address
    [T]: TWR slot number

    Delete Tag ~ Reset Tag
    $delTag,[A],[T],*
    [A]: Tag address
    [T]: TWR slot number

    Delete Anchor ~ Delete Anchor
    $delAn,[A],[B],*
    [A]: Anchor address (2 byte)
    [B]: BCN slot number (1 byte)

    Config superframe
    $cfgSf,[N],*
    [N]: SF total

    Config Anchor location
    $cfgLoc,[A],[B],[X],[Y]
    [A]: Anchor address (2 byte)
    [B]: BCN slot number (1 byte)
    [X]: X location (4 byte)
    [Y]: Y location (4 byte)

    Report Tag lcation
    $Loc,[A1][X][Y][A2][X][Y]...,*

    Start/Stop
    $start* (Cho phep tat ca cac anchor bat dau hoat dong)
    $stop* (Dung lai toan bo mang)
*/

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
} __attribute__((packed)) payload_t;

typedef struct
{
    uint8_t address[2];
    uint8_t sf_num;
} __attribute__((packed)) cfg_superframe_t;

typedef struct
{
    uint8_t bcnSlotTotal;
    uint8_t twrSlotTotal;
    uint8_t sfNumberTotal;
    cfg_anchor_t anchorInfo;
} __attribute__((packed)) info_t;

typedef payload_t cfg_tag_t;
typedef payload_t del_anchor_t;
typedef payload_t del_tag_t;
typedef uint8_t cmd_start_t;
typedef uint8_t cmd_stop_t;

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

#endif
