#include "can.h"
#include <stm32f0xx.h>
#include "stm32f0_helpers.h"
#include "nocan.h"
#include "gpio.h"

/************************************************
 *
 * Extern
 *
 */

uint8_t *can_rx_buffer;
uint8_t can_tx_buffer[CAN_BUFFER_LEN];
uint8_t can_filter_buffer[CAN_FILTER_LEN];

/************************************************
 * 
 *
 * Message assembly pipeline
 *
 *
 */

typedef struct __attribute__((packed)) {
    uint32_t hash;
    uint8_t age;
    /**/
    uint8_t data[4+1+64];
} can_buffer_t;

/**** RX BUFFER *********************************/

#define BUFFER_LOCKED  0x00
#define BUFFER_FREE    0xFF

#define NORMALIZED_EID(x) ((x)>>3)

#define MUX_COUNT 7         // Must be (2^n-1) 
can_buffer_t can_mux[MUX_COUNT];
can_buffer_t *can_rx_queue[MUX_COUNT+1];
uint32_t can_rx_queue_head;
uint32_t can_rx_queue_tail;

int can_shift_rx_buffer(void)
{
    if (can_rx_queue_tail == can_rx_queue_head)
        return -1;

    can_rx_queue[can_rx_queue_head]->age = BUFFER_FREE; // unlock the buffer

    can_rx_queue_head = (can_rx_queue_head+1) & MUX_COUNT;

    if (can_rx_queue_tail == can_rx_queue_head) // empty?
    {
        NOCAN_STATUS_CLEAR(NOCAN_STATUS_RX_PENDING);
        gpio_set_rx_int();
        // do nothing about *nocan_rx_buffer*
    }
    else
    {
        NOCAN_STATUS_SET(NOCAN_STATUS_RX_PENDING);
        gpio_clear_rx_int();
        can_rx_buffer = can_rx_queue[can_rx_queue_head]->data;
    }
    return 0;
}

static inline int _can_push_rx_buffer(can_buffer_t *buffer)
{
    uint32_t next = (can_rx_queue_tail+1) & MUX_COUNT;

    if (next == can_rx_queue_head)
        return -1;

    buffer->age = BUFFER_LOCKED; // lock the buffer

    if (can_rx_queue_tail == can_rx_queue_head) // are we empty? if yes set can_rx_buffer, otherwise leave existing value unmodified.
        can_rx_buffer = buffer->data;

    can_rx_queue[can_rx_queue_tail] = buffer;

    can_rx_queue_tail = next;

    NOCAN_STATUS_SET(NOCAN_STATUS_RX_PENDING);
    gpio_clear_rx_int(); 
    return 0;
}

#define nocan_mux_release_buffer(buffer) ((buffer)->age = BUFFER_FREE)

static int mux_search(uint32_t eid)
{
    int i;
    uint32_t hash = EID_GET_HASH(eid);

    // Age all buffers, except empty ones which are marked with age==BUFFER_FREE 
    // or the ones that are marked as complete with age==0x00. 
    // Eventually unused buffers go back to BUFFER_FREE (free state)
    for (i=0; i<MUX_COUNT; i++)
    {
        if (can_mux[i].age<BUFFER_FREE && can_mux[i].age!=BUFFER_LOCKED) 
            can_mux[i].age++;
    }

    for (i=0; i<MUX_COUNT; i++)
    {
        if (can_mux[i].hash==hash && can_mux[i].age<BUFFER_FREE && can_mux[i].age!=BUFFER_LOCKED)
        {
            can_mux[i].age = 1; // refresh
            return i;
        }
    }

    // else: find any buffer that, either
    // - either is free (age==BUFFER_FREE), 
    // - or has been there for the longest.
    int oldest = -1;
    uint8_t age = 0;
    for (i=0;i<MUX_COUNT;i++)
    {
        if (can_mux[i].age>age)
        {
            age = can_mux[i].age;
            oldest = i;
            if (age==BUFFER_FREE) break;
        }
    }

    if (oldest>=0) {
        can_mux[oldest].age = 1;
        can_mux[oldest].hash = hash;
        can_mux[oldest].data[0] = (uint8_t)(eid>>24);
        can_mux[oldest].data[1] = (uint8_t)(eid>>16);
        can_mux[oldest].data[2] = (uint8_t)(eid>>8);
        can_mux[oldest].data[3] = (uint8_t)(eid);
        can_mux[oldest].data[4] = 0; // dlc
        return oldest;
    }

    return -1; 
}

static int can_mux_process(const can_packet_t *msg)
{
    uint32_t eid = NORMALIZED_EID(msg->eid);
    int bindex = mux_search(eid);
    can_buffer_t *buf;


    if (bindex<0) 
        return nocan_error_set(NOCAN_STATUS_ERROR_RX_OVERFLOW);

    buf = can_mux + bindex;

    if (buf->data[4]>56)
    {
        nocan_mux_release_buffer(buf);
        return nocan_error_set(NOCAN_STATUS_ERROR_RX_MESSAGE);
    }

    if ((eid&EID_FIRST_PACKET_MASK)!=0 && buf->data[4]>0)
    {
        nocan_mux_release_buffer(buf);
        return nocan_error_set(NOCAN_STATUS_ERROR_RX_MESSAGE);
    }

    uint8_t pos = buf->data[4]+5;
    buf->data[pos++] = msg->data_l;
    buf->data[pos++] = msg->data_l >> 8;
    buf->data[pos++] = msg->data_l >> 16;
    buf->data[pos++] = msg->data_l >> 24;
    buf->data[pos++] = msg->data_h;
    buf->data[pos++] = msg->data_h >> 8;
    buf->data[pos++] = msg->data_h >> 16;
    buf->data[pos++] = msg->data_h >> 24;
    buf->data[4]+=msg->dlc;

    if ((eid&EID_LAST_PACKET_MASK)!=0)
    {
        _can_push_rx_buffer(buf);
    }
    return NOCAN_OK;
}

static int can_sys_process(const can_packet_t *packet)
{
    int i;
    uint32_t eid = NORMALIZED_EID(packet->eid);
    int8_t eid_param = (int8_t)EID_GET_PARAMETER(eid);
    uint8_t eid_func = EID_GET_FUNCTION(eid);

    if (eid_func == NOCAN_SYS_NODE_BOOT_REQUEST)
    {
        if ((eid_param&0x01)!=0)
        {
            gpio_clear_reset();
            nocan_close();
            for (uint32_t count=0;count<10000;count++) asm("nop");
            gpio_set_reset();
        }
        if ((eid_param&0x02)!=0)
        {
            NVIC_SystemReset();
        }
        return NOCAN_OK;
    }
    if (eid_func == NOCAN_SYS_NODE_PING)
    {
        // NOTE: sys we want to process a sys messages and send a response
        // we use another CAN mailbox (mbox 1) to avoid conflicts with ongoing 
        // transmits (in mbox 0).

        return can_send_ping_ack(EID_GET_NODE_ID(eid));
    }

    return can_mux_process(packet);
}


/**** TX BUFFER *********************************/

uint32_t can_tx_eid;
volatile uint32_t can_tx_head;
volatile uint32_t can_tx_tail;

int can_send_frame(void) 
{
    uint32_t eid = can_tx_eid;
    uint8_t dlc;
    #define transmit_mailbox 0

    /*
    if ((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
    {
        transmit_mailbox = 0;
    }
    else if ((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
    {
        transmit_mailbox = 1;
    }
    else if ((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
    {
        transmit_mailbox = 2;
    }
    else
    {
        return -1;
    }
    */
    // check if transmit mailbox 0 is avaiable
    if ((CAN->TSR&CAN_TSR_TME0) != CAN_TSR_TME0)
        return -1;

    if (can_tx_head==5)
        eid |= EID_FIRST_PACKET_MASK;     // FIRST BIT

    if (can_tx_tail - can_tx_head<=8)
    {
        eid |= EID_LAST_PACKET_MASK;     // LAST BIT
        dlc = can_tx_tail - can_tx_head;
    }
    else
    {
        dlc = 8;
    }

    const uint8_t *data = can_tx_buffer + can_tx_head;

    CAN->sTxMailBox[transmit_mailbox].TIR = (eid<<3) | (1<<2); 
    CAN->sTxMailBox[transmit_mailbox].TDTR &= (uint32_t)0xFFFFFFF0;
    CAN->sTxMailBox[transmit_mailbox].TDTR |= dlc;
    CAN->sTxMailBox[transmit_mailbox].TDLR = (((uint32_t)data[3] << 24) |
            ((uint32_t)data[2] << 16) |
            ((uint32_t)data[1] << 8) |
            ((uint32_t)data[0]));
    CAN->sTxMailBox[transmit_mailbox].TDHR = (((uint32_t)data[7] << 24) |
            ((uint32_t)data[6] << 16) |
            ((uint32_t)data[5] << 8) |
            ((uint32_t)data[4]));

    can_tx_head += dlc;
    CAN->sTxMailBox[transmit_mailbox].TIR |= 1; // (1<<0) is TXRQ flag (transmit request)
    
    return 0;
}
  
int can_commit_tx_buffer(void)
{
    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_TX_PENDING)!=0)
        return -1;

    NOCAN_STATUS_SET(NOCAN_STATUS_TX_PENDING);

    can_tx_eid = ((uint32_t)can_tx_buffer[0] << 24) 
        | ((uint32_t)can_tx_buffer[1] << 16) 
        | ((uint32_t)can_tx_buffer[2] << 8) 
        | ((uint32_t)can_tx_buffer[3]) 
        ;
    can_tx_head = 5;
    can_tx_tail = 5+can_tx_buffer[4];
    return can_send_frame();
}

int can_sys_send(uint8_t node_id, uint8_t fn, uint8_t param, uint8_t *data, uint8_t data_len) 
{
    uint32_t eid, i;

    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_TX_PENDING)!=0)
        return -1;
    can_tx_eid = ((uint32_t)node_id<<21) | EID_SYS_MASK | ((uint32_t)fn<<8) | (uint32_t)param;
    for (i=0; i<data_len; i++)
        can_tx_buffer[5+i] = data[i];
    can_tx_head = 5;
    can_tx_tail = 5 + data_len;
    return can_send_frame();
}

/** Response to PING **/

int can_send_ping_ack(uint8_t node_id) 
{
    uint32_t eid;

    // check if transmit mailbox 1 is avaiable
    if ((CAN->TSR&CAN_TSR_TME1) != CAN_TSR_TME1)
        return -1;

    // make a simple ping ack packet 
    eid = EID_FIRST_PACKET_MASK | ((uint32_t)node_id<<21) | EID_LAST_PACKET_MASK | EID_SYS_MASK | ((uint32_t)NOCAN_SYS_NODE_PING_ACK<<8);

    CAN->sTxMailBox[1].TIR = (eid<<3) | (1<<2); 
    CAN->sTxMailBox[1].TDTR &= (uint32_t)0xFFFFFFF0;
    CAN->sTxMailBox[1].TDTR |= 0;
    CAN->sTxMailBox[1].TDLR = 0;
    CAN->sTxMailBox[1].TDHR = 0;
    CAN->sTxMailBox[1].TIR |= 1; // (1<<0) is TXRQ flag (transmit request)
    
    return 0;
}
  

/************************************************
 * 
 *
 * Message filtering
 *
 *
 */

#define FIFO_0 0
#define FIFO_1 1

static void can_filter_set(uint32_t filter_id, uint32_t filter, uint32_t mask, int fifo)
{
    // Init mode
    CAN->FMR |= CAN_FMR_FINIT;

    // Deactivate filter
    CAN->FA1R &= ~(1<<filter_id);

    // 32 bitr scale for filter
    CAN->FS1R |= (1<<filter_id);

    CAN->sFilterRegister[filter_id].FR1 = filter;
    CAN->sFilterRegister[filter_id].FR2 = mask;
    
    // Id/Mask mode (0)
    CAN->FM1R &= ~(1<<filter_id);

    // set fifo
    if (fifo==FIFO_0)
        CAN->FFA1R &= ~(1<<filter_id);
    if (fifo==FIFO_1)
        CAN->FFA1R |= (1<<filter_id);

    // activate
    CAN->FA1R |= (1<<filter_id);
    
    // Leave filter init mode
    CAN->FMR &= ~CAN_FMR_FINIT;
}

static int can_filter_get(uint32_t filter_id, uint32_t *filter, uint32_t *mask, int *fifo)
{
    *filter = CAN->sFilterRegister[filter_id].FR1;
    *mask   = CAN->sFilterRegister[filter_id].FR2;
    *fifo   = ((CAN->FFA1R >> filter_id)&1);
    return (int)((CAN->FA1R>>filter_id)&1);
}

static void can_filter_deactivate(uint32_t filter_id)
{
    // Init mode
    CAN->FMR |= CAN_FMR_FINIT;

    // Deactivate filter
    CAN->FA1R &= ~(1<<filter_id);

    // Leave filter init mode
    CAN->FMR &= ~CAN_FMR_FINIT;
}

void can_node_filter(uint8_t node_id)
{
    can_filter_set(13, ((uint32_t)node_id&0x7F)<<24 | 0x200004, 0x7F200004, FIFO_1);
            // 0x200000 is sys bit, 0x04 is eid bit
}

void can_commit_filter(uint8_t filter)
{
    uint16_t channel_id;
    
    if (filter>=CAN_FILTER_COUNT) return;

    channel_id = ((int16_t)can_filter_buffer[3*filter+1]<<8) | ((int16_t)can_filter_buffer[3*filter+2]);
    if (channel_id!=0xFFFF)
    {
        can_filter_set(filter, ((uint32_t)channel_id<<3) | 0x0004, 0x0007FFF8 | 0x0004, FIFO_0);
    }
    else
    {
        can_filter_deactivate(filter);
    }
    can_filter_buffer[3*filter] = 0xAA;
}

/************************************************
 * 
 *
 * CAN core processing
 *
 *
 */

void CEC_CAN_IRQHandler(void)
{
    can_packet_t packet;

    if((CAN->RF0R & CAN_RF0R_FMP0)!=0)/* check if a packet is filtered and received by FIFO 0 */
    {   
        can_mux_process((can_packet_t *)CAN->sFIFOMailBox+0);
        CAN->RF0R |= CAN_RF0R_RFOM0; // release the fifo
        NOCAN_REGS.CAN_RX_COUNT++;
    }
    
    if((CAN->RF1R & CAN_RF1R_FMP1)!=0)/* check if a packet is filtered and received by FIFO 1 */
    {   
        can_sys_process((can_packet_t *)CAN->sFIFOMailBox+1);
        CAN->RF1R |= CAN_RF1R_RFOM1; // release the fifo
        NOCAN_REGS.CAN_RX_COUNT++;
    }

    if((CAN->TSR & CAN_TSR_RQCP0)!=0) /* check if request completed for mailbox 0 */
    {
        CAN->TSR |= CAN_TSR_RQCP0; // release flag
        if (can_tx_head < can_tx_tail)
        {
            can_send_frame();
        }
        else
        {
            NOCAN_STATUS_CLEAR(NOCAN_STATUS_TX_PENDING); 
            gpio_set_tx_int(); 
        }
        NOCAN_REGS.CAN_TX_COUNT++;
    } 
}

void can_close(void)
{
    NVIC_DisableIRQ(CEC_CAN_IRQn);
    // enable CAN1 RESET State
    RCC->APB1RSTR |= RCC_APB1RSTR_CANRST; 
    // leave CAN1 RESET State
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CANRST;
}

int can_open(void)
{
    /* REMAP PINS */

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

    /* CAN GPIOs configuration **************************************************/

    /* Enable GPIO clock */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* Connect CAN pins to AF4 */
    GPIO_CONFIGURE_ALTERNATE_FUNCTION(GPIOA, 11, GPIO_AF_4);
    GPIO_CONFIGURE_ALTERNATE_FUNCTION(GPIOA, 12, GPIO_AF_4);


    /* CAN configuration ********************************************************/  
    
    
    /* Enable CAN clock */
    RCC->APB1ENR &= ~RCC_APB1ENR_CANEN;
    RCC->APB1ENR |= RCC_APB1ENR_CANEN;

    /* CAN cell init */
    CAN->MCR &=~ CAN_MCR_SLEEP;  // Exit sleep mode
   
    /* CAN register init */
    CAN->MCR |= CAN_MCR_INRQ; /* (1) */
    while((CAN->MSR & CAN_MSR_INAK)!=CAN_MSR_INAK) {}

    
    /* Automatic bus off management: 
     * The Bus-Off state is left automatically by hardware once 128 occurrences of 11 recessive
     * bits have been monitored.
     */
    CAN->MCR |= CAN_MCR_ABOM;

#ifdef USE_FASTER_CAN

#define CAN_SJW_1tq 0
#define CAN_BS1_11tq 10
#define CAN_BS2_4tq 3
#define CAN_PRESCALE 12

    /* CAN Baudrate = 250K (CAN clocked at 48 MHz) */
    // (48000000/250000)/12 => 16 => (1+SJW)+(1+BS1)+(1+BS2) where (1+SJW)+(1+BS1)/16==75%

    CAN->BTR = (0<<30) // normal mode 
        | (CAN_SJW_1tq << 24)
        | (CAN_BS1_11tq << 16)
        | (CAN_BS2_4tq << 20)
        | (CAN_PRESCALE-1)               // prescaler
        ;

#else

#define CAN_SJW_1tq 0
#define CAN_BS1_11tq 10
#define CAN_BS2_4tq 3 
#define CAN_PRESCALE 24

    /* CAN Baudrate = 125k (CAN clocked at 48 MHz) */
    // (48000000/125000)/24 => 16 => (1+SJW)+(1+BS1)+(1+BS2) where (1+SJW)+(1+BS1)/16==75%
    CAN->BTR = (0 << 30) // normal mode
        | (CAN_SJW_1tq << 24)
        | (CAN_BS1_11tq << 16)
        | (CAN_BS2_4tq << 20)
        | (CAN_PRESCALE-1)              // prescaler
        ;

#endif

    /* Leave Init mode */
    CAN->MCR &= ~CAN_MCR_INRQ;
    while((CAN->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) {}

   
    /* Enable FIFO 0 and FIFO 1 message pending Interrupts 
     * Enable transmit mailbox empty Interrupt is set in can_tx_send()
     */
    CAN->IER = CAN_IER_FMPIE0
        | CAN_IER_FMPIE1
        | CAN_IER_TMEIE
        ;


    for (int i=0;i<MUX_COUNT;i++)
        nocan_mux_release_buffer(can_mux+i);

    can_rx_queue_head = 0;
    can_rx_queue_tail = 0;
    can_rx_queue[0] = can_mux;
    can_rx_buffer = can_mux[0].data;

    can_tx_head = 0;
    can_tx_tail = 0;

    /* CAN filter init */
    for (int i=0;i<CAN_FILTER_COUNT;i++) 
    {
        can_filter_deactivate(i);
        can_filter_buffer[i*3] = 0xFF;
        can_filter_buffer[i*3+1] = 0xFF;
        can_filter_buffer[i*3+2] = 0xFF;
    }
    can_node_filter(0);

    NOCAN_REGS.CAN_RX_COUNT = 0;
    NOCAN_REGS.CAN_TX_COUNT = 0;
 
    /* NVIC configuration *******************************************************/
    NVIC_SetPriority(CEC_CAN_IRQn, 1); 
    NVIC_EnableIRQ(CEC_CAN_IRQn);
    return 1;
}


