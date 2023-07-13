#ifndef _MAC_H_
#define _MAC_H_

#include <stdint.h>

#define MAC802154_TYPE_BEACON 0
#define MAC802154_TYPE_DATA 1
#define MAC802154_TYPE_ACK 2
#define MAC802154_TYPE_CMD 3
#define MAC802154_TYPE_RESERVED 4
#define MAC802154_HEADER_LENGTH 9
#define MAC802154_PAN_ID 0x3838

typedef struct packet_s
{
    union
    {
        uint16_t fcf; // frame control field
        struct
        {
            uint16_t type : 3;
            uint16_t security : 1;
            uint16_t framePending : 1;
            uint16_t ack : 1;  // the recepient device should no be sent acknowledgement frame (=0)
            uint16_t ipan : 1; // source and destination addresses are present (=1)
            uint16_t reserved : 3;
            uint16_t destAddrMode : 2; // destination address field is an extended address (=2)
            uint16_t version : 2;      // frame version field, indicate an IEEE 802.15.4 frame (=1)
            uint16_t srcAddrMode : 2;  // source address field is an extended address (=2)
        } fcf_s;
    };
    uint8_t seq;
    uint16_t pan;
    uint8_t destAddress[2];
    uint8_t sourceAddress[2];
    uint8_t payload[128];
} __attribute__((packed)) packet_t;

#define MAC80215_PACKET_INIT(packet, TYPE) packet.fcf_s.type = (TYPE); \
    packet.fcf_s.security = 0;             \
    packet.fcf_s.framePending = 0;         \
    packet.fcf_s.ack = 0;                  \
    packet.fcf_s.ipan = 1;                 \
    packet.fcf_s.destAddrMode = 2;         \
    packet.fcf_s.version = 1;              \
    packet.fcf_s.srcAddrMode = 2;		   \
    packet.pan = MAC802154_PAN_ID;

#endif
