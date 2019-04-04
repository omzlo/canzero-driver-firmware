#include <stm32f0xx.h>
#include "can.h"
#include "nocan.h"
#include "gpio.h"
#include "systick.h"
#include "spi_slave.h"
#include "chip_options.h"
#include "version.h"

nocan_registers_t NOCAN_REGS;

/* * * * 
 * The pseudo hash below is loosely based on MD5.
 * It aims the create a unique 8 byte ID from a 12 unique ID
 * with low probability of collision.
 * The previous approach of selecting certain bits for the 12 byte
 * ID to create an 8 byte ID had an 2-3% collissions, which is not
 * very big but could still become problematic.
 */

uint16_t K[24] = {
    0xa478, 0x0faf, 0x98d8, 0x1122,
    0x2562, 0x105d, 0xcde6, 0xe905,
    0x3942, 0xea44, 0x7ec6, 0xd039,
    0x2244, 0x59c3, 0x7e4f, 0x7e82,
    0xb756, 0xc62a, 0xf7af, 0x7193,
    0xb340, 0x1453, 0x07d6, 0xa3f8
};

int s[16] = { 7, 12,  1, 6,
              5,  9, 14, 2,
              4, 11,  0, 3,
              8, 10, 15, 13 };

static uint16_t lrot(uint16_t v, int r)
{
    return (v<<r) | (v>>(16-r));
}

static void pseudo_hash(uint16_t *src, uint16_t *dst)
{
    uint16_t A, B, C, D;

    A = 0x2301;
    B = 0xab89;
    C = 0xdcfe;
    D = 0x5476;

    for (int i=0; i<16; i++)
    {
        uint16_t R, p;

        if (i<6) {
            R = (B & C) | ((~B) & D);
            p = i;
        }
        else if (i<8) {
            R = (B & D) | (C & (~D));
            p = (5*i+1)%6;
        }
        else if (i<12) {
            R = B ^ C ^ D;
            p = (3*i+5)%6;
        }
        else {
            R = C ^ (B | (~D));
            p = (2*i)%6;
        }
        R = R + A + K[i] + src[p];
        A = D;
        D = C;
        C = B;
        B = B + lrot(R, s[i]);
    }
    dst[0] = A;
    dst[1] = B;
    dst[2] = C;
    dst[3] = D;
}


/* * * */

int nocan_error_set(uint8_t ecode)
{
    NOCAN_STATUS_SET(ecode);
    if (ecode==0)
        return 0;
    return -1;
}

void nocan_error_clear(void)
{
    NOCAN_STATUS_CLEAR(0xF0);   // upper nibble is for errors.
}


uint8_t _compact_number(uint8_t n)
{
    if (n>='0' && n<='9') return n-'0';
    return 0xF;
}

int nocan_open(void)
{
    int i;
    uint8_t *p=(uint8_t *)&NOCAN_REGS;

    can_open();

    gpio_set_tx_int();
    gpio_set_rx_int();

    for (i=0;i<sizeof(nocan_registers_t);i++)
        p[i]=0;

    // Normally we would only need to do nocan_status_clear(0xFF).
    // However, if we call this within a SPI call, this will remove the busy signal and create issues.
    // So we introduce the possibility to preserve NOCAN_STATUS_BUSY if needed.
    
   
    /* src: CHIP_UDID (12 bytes), dst: NOCAN_REGS.UDID (8 bytes) */ 
    pseudo_hash((uint16_t *)CHIP_UDID, (uint16_t *)NOCAN_REGS.UDID);
    for (i=0; i<12; i++)
        NOCAN_REGS.REAL_UDID[i] = CHIP_UDID[i];

    NOCAN_REGS.signature[0] = 'N';
    NOCAN_REGS.signature[1] = 'C';
    NOCAN_REGS.signature[2] = 'A';
    NOCAN_REGS.signature[3] = 'N';
    
    NOCAN_REGS.VERSION = STM32_DRIVER_VERSION;

    return 0;
}

void nocan_close(void)
{
    can_close();
}

const char *nocan_status_string(void) 
{
    static char status[20];
    int pos = 0;

    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_RX_PENDING)!=0)
    {
        status[pos++] = '+';
        status[pos++] = 'R';
        status[pos++] = 'X';
    }

    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_TX_PENDING)!=0)
    {
        status[pos++] = '+';
        status[pos++] = 'T';
        status[pos++] = 'X';
    }

    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_LED)!=0)
    {
        status[pos++] = '+';
        status[pos++] = 'L';
        status[pos++] = 'D';
    }

    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_ERROR_TX_OVERFLOW)!=0)
    {
        status[pos++] = '!';
        status[pos++] = 'T';
        status[pos++] = 'O';
    }

    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_ERROR_RX_OVERFLOW)!=0)
    {
        status[pos++] = '!';
        status[pos++] = 'R';
        status[pos++] = 'O';
    }

    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_ERROR_RX_MESSAGE)!=0)
    {
        status[pos++] = '!';
        status[pos++] = 'R';
        status[pos++] = 'M';
    }
    status[pos++] = '\0';
    return status;
}
