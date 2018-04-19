#ifndef _DEBUG_EVENT_H_
#define _DEBUG_EVENT_H_

#define DEVENT_START    0x00
#define DEVENT_MARK_1   0x01
#define DEVENT_MARK_2   0x02
#define DEVENT_MARK_3   0x03
#define DEVENT_CAN_SEND 0xD0
#define DEVENT_CAN_RECV 0xC0

#ifdef DEBUG_EVENTS

#include <stdint.h>

typedef struct {
    uint32_t h;
    uint32_t l;
} debug_event_t;

void debug_init(void);

int debug_push(uint32_t h, uint32_t l);

int debug_pop(uint32_t *h, uint32_t *l);

#else

#define debug_init()    /*void*/
#define debug_push(h,l) (0) 
#define debug_pop(h,j)  (-1)

#endif

#endif
