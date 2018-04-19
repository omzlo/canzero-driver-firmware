#include <stm32f0xx.h>
#include "can.h"
#include "nocan.h"
#include "gpio.h"
#include "systick.h"
#include "spi_slave.h"
#include "chip_options.h"

nocan_registers_t NOCAN_REGS;

/* * * */

/*
//void nocan_status_set(uint8_t status)
{
    __disable_irq();
    NOCAN_REGS.STATUS |= status;
    __enable_irq();
    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_TX_PENDING)!=0)
    {
        gpio_write_tx_int(LOW);
    }
    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_RX_PENDING)!=0)
    {
        gpio_write_rx_int(LOW);
    }
    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_LED)!=0)
    {
        gpio_write_led(HIGH);
    }
}

//void nocan_status_clear(uint8_t status)
{
    __disable_irq();
    NOCAN_REGS.STATUS &= ~status;
    __enable_irq();
    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_TX_PENDING)==0)
    {
        gpio_write_tx_int(HIGH);
    }
    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_RX_PENDING)==0)
    {
        gpio_write_rx_int(HIGH);
    }
    if ((NOCAN_REGS.STATUS & NOCAN_STATUS_LED)==0)
    {
        gpio_write_led(LOW);
    }
}
*/

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
    NOCAN_REGS.UDID[0] = CHIP_UDID[0];
    NOCAN_REGS.UDID[1] = CHIP_UDID[2];
    NOCAN_REGS.UDID[2] = CHIP_UDID[4];
    NOCAN_REGS.UDID[3] = CHIP_UDID[5];
    NOCAN_REGS.UDID[4] = CHIP_UDID[6];
    NOCAN_REGS.UDID[5] = CHIP_UDID[7];
    NOCAN_REGS.UDID[6] = (_compact_number(CHIP_UDID[8])<<4) | _compact_number(CHIP_UDID[9]);
    NOCAN_REGS.UDID[7] = (_compact_number(CHIP_UDID[10])<<4) | _compact_number(CHIP_UDID[11]);

    NOCAN_REGS.signature[0] = 'N';
    NOCAN_REGS.signature[1] = 'C';
    NOCAN_REGS.signature[2] = 'A';
    NOCAN_REGS.signature[3] = 'N';
    //NOCAN_REGS.GUARD = 0x42;

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
