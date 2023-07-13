#ifndef _UART_TYPES_H_
#define _UART_TYPES_H_

typedef enum
{
    WAIT_START_BYTE,
    WAIT_END_BYTE
} rxState_t;

/*
    Config Anchor ~ Add Anchor
    $cfgAn,[A],[B],[X],[Y],[SF],*
    [A]: Anchor address (2 byte)
    [B]: BCN slot number (1 byte)
    [X]: X location (4 byte)
    [Y]: Y location (4 byte)
    [SF]: SF total (1 byte)

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
    $tagLog,[A1][X][Y],[A2][X][Y],...,*

    Start/Stop
    $start* (Cho phep tat ca cac anchor bat dau hoat dong)
    $stop* (Dung lai toan bo mang)
*/

typedef struct
{
    uint8_t anchorAddr[2];
    int bcnSlotNum;
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
    int sfTotal;
} cfgAnchor_t;

typedef struct
{
    uint8_t tagAddr[2];
    uint8_t twrSlotNum;
} provTag_t;

typedef struct
{
    uint8_t tagAddr[2];
    uint8_t twrSlotNum;
} delTag_t;

#endif