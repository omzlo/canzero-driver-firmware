#ifndef _CAN_H_
#define _CAN_H_

#include <stdint.h>



typedef struct __attribute__((packed)) {
    uint32_t eid;
    uint32_t dlc;
    uint32_t data_l;
    uint32_t data_h;
} can_packet_t;


#define CAN_BUFFER_LEN (4+1+64)
#define CAN_FILTER_COUNT 12
#define CAN_FILTER_LEN (CAN_FILTER_COUNT*3)

extern uint8_t *can_rx_buffer;
extern uint8_t can_tx_buffer[CAN_BUFFER_LEN];
extern uint8_t can_filter_buffer[CAN_FILTER_LEN];

int can_open(void);

void can_close(void);

int can_shift_rx_buffer();

int can_commit_tx_buffer();

void can_commit_filter(uint8_t filter);

void can_node_filter(uint8_t filter);

int can_sys_send(uint8_t node_id, uint8_t fn, uint8_t param, uint8_t *data, uint8_t data_len);

int can_send_ping_ack(uint8_t node_id);

#endif
