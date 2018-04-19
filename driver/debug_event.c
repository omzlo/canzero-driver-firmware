#ifdef DEBUG_EVENTS

#include "debug_event.h"

#define DQUEUE_SIZE 16
#define DQUEUE_NEXT(a) (((a)+1)&(DQUEUE_SIZE-1))

debug_event_t dqueue[DQUEUE_SIZE];
uint32_t dqueue_head;
uint32_t dqueue_tail;

void debug_init(void)
{
    dqueue_head = 0;
    dqueue_tail = 0;
}

int debug_push(uint32_t h, uint32_t l)
{
    if (DQUEUE_NEXT(dqueue_tail)==dqueue_head)
        return -1;
    dqueue[dqueue_tail].h = h;
    dqueue[dqueue_tail].l = l;
    dqueue_tail = DQUEUE_NEXT(dqueue_tail);
    return 0;
}

int debug_pop(uint32_t *h, uint32_t *l)
{
    if (dqueue_head == dqueue_tail)
        return -1;
    *h = dqueue[dqueue_head].h;
    *l = dqueue[dqueue_head].l;
    dqueue_head = DQUEUE_NEXT(dqueue_head);
    return 0;
}

#warning "DEBUG EVENTS are embedded in the code."

#endif
