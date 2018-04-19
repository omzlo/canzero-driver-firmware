#ifndef _NOCAN_H_
#define _NOCAN_H_

#include "can.h"

/*
    EID format used:
    npos pos size    description
    ------------------------
    28   0   1       first_packet_flag   (0=>not the fist packet in a block, 1=> is the first packet)
    21   1   7       node_id      
    20   8   1       last_packet_flag    (0=>not the last packet in a block, 1=> is not the last packet)
    19   9   1       reserved 
    18   10  1       sys_flag

    -- if sys_flag == 0 then
    16   11  2       reserved
     0   13  16      channel_id 

    -- if sys_flag == 1 then
    16   11  2       reserved
     8   13  8       function
     0   21  8       parameter
       
    ------------------------
             29      total length
*/ 

#define EID_FIRST_PACKET_MASK       (1<<28)
#define EID_NODE_ID_MASK            (0x7F<<21)
#define EID_LAST_PACKET_MASK        (1<<20)
#define EID_SYS_MASK                (1<<18)
#define EID_CHANNEL_ID_MASK         (0xFFFF)
#define EID_FUNCTION_MASK           (0xFF00)
#define EID_PARAMETER_MASK          (0x00FF)

#define EID_GET_NODE_ID(eid)        (((eid)&EID_NODE_ID_MASK)>>21)
#define EID_GET_CHANNEL_ID(eid)     (((eid)&EID_CHANNEL_ID_MASK))
#define EID_GET_FUNCTION(eid)       (((eid)&EID_FUNCTION_MASK)>>8)
#define EID_GET_PARAMETER(eid)      (((eid)&EID_PARAMETER_MASK))
#define EID_GET_HASH(eid)           ((eid)&(EID_NODE_ID_MASK|EID_SYS_MASK|EID_CHANNEL_ID_MASK))

#define NOCAN_SYS_ADDRESS_REQUEST          1
#define NOCAN_SYS_NODE_BOOT_REQUEST        6
#define NOCAN_SYS_NODE_BOOT_ACK            7
#define NOCAN_SYS_NODE_PING                8
#define NOCAN_SYS_NODE_PING_ACK            9


#define NOCAN_OK                     0
#define NOCAN_ERROR                 -1

int nocan_open(void);

void nocan_close(void);

int nocan_error_set(uint8_t ecode);

void nocan_error_clear(void);

#define NOCAN_STATUS_SET(status)    do { \
    __disable_irq(); \
    NOCAN_REGS.STATUS |= (status); \
    __enable_irq(); \
} while (0)

#define NOCAN_STATUS_CLEAR(status)  do { \
    __disable_irq(); \
    NOCAN_REGS.STATUS &= ~(status); \
    __enable_irq(); \
} while (0)

#define NOCAN_STATUS_RX_PENDING         0x01 
#define NOCAN_STATUS_TX_PENDING         0x02
#define NOCAN_STATUS_LED                0x04
#define NOCAN_STATUS_ERROR_TX_OVERFLOW  0x10
#define NOCAN_STATUS_ERROR_RX_OVERFLOW  0x20
#define NOCAN_STATUS_ERROR_RX_MESSAGE   0x40

typedef struct __attribute__((packed)) {
    uint8_t signature[4];
    volatile uint8_t STATUS;         // status register
    uint8_t NODE_ID;
    uint8_t UDID[8];
    uint32_t CAN_RX_COUNT;
    uint32_t CAN_TX_COUNT;
} nocan_registers_t;

extern nocan_registers_t NOCAN_REGS;

const char *nocan_status_string(void);


#endif
