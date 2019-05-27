#include <stm32f0xx.h>
#include "spi_slave.h"
#include "nocan.h"
#include "gpio.h"

#define SPI_WRITE(b) (*(uint8_t *)&(SPI1->DR) = (uint8_t)(b))

#define SPI_READ() ((uint8_t)(SPI1->DR))


// typedef uint8_t (*spi_transfer_cb)(uint8_t, uint8_t);  // (offset,r_value) -> w_value
// SPI_SS on PA4
// SPI1_SCK on PA5
// SPI1_MISO on PA6
// SPI1_MOSI on PA7

void spi_slave_configure_EXTI(void)
{
  /* (1) PA4 as source input */
  /* (2) unmask port 0 */
  /* (3) Rising edge */
  /* (4) Set priority */
  /* (5) Enable EXTI0_1_IRQn */

  SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~SYSCFG_EXTICR2_EXTI4) | SYSCFG_EXTICR2_EXTI4_PA; /* (1) */ 
  //  SYSCFG->EXTICR[0] => PA0, PB0, PC0, ... PF0 as specified in bits [0:3] of  SYSCFG_EXTICR1 
  //                    => PA1, PB1, PC1, ... PF1 as specified in bits [4:7]
  //                    ...
  //                    => PA3, PB3, PC3, ... PF3 as specified in bits [12:15]
  //
  //  SYSCFG->EXTICR[1] => PA4, PB4, PC4, ... PF4 as specified in bits [0:3] of  SYSCFG_EXTICR2 
  //  
    
  
  
  EXTI->IMR |= EXTI_IMR_MR4; /* (2) */
  // Interrupt request form line _0_ is unmasked (1)
  // SYSCFG->EXTICR selects the letter PA, PB, ... PF and here we select one or more pins 
  // for the letter  (incorrect)

  // Rising edge on line _0_
  // EXTI->FTSR for falling edge
  EXTI->RTSR |= EXTI_RTSR_TR4; /* (3) */ 
  EXTI->FTSR |= EXTI_FTSR_TR4;

  NVIC_SetPriority(EXTI4_15_IRQn, 0); /* (4) */ 
  // EXTI0_1 covers interrupts on pins Px0 and Px1
  // EXTI2_3 covers interrupts on pins Px2 and Px3
  // EXTI4_15 coverts interrupts on pins Px4, Px5, Px6, ..., Px15
  // Priority 0 is the highest (as here), priority 3 is the lowest 
  //=// NVIC_EnableIRQ(EXTI0_1_IRQn); /* (5) */ 
  NVIC_EnableIRQ(EXTI4_15_IRQn); /* (5) */ 
}

#define SPI_OK_BYTE 0x80
#define SPI_MORE_BYTE 0xA0
#define SPI_ERR_BYTE 0xFF

static uint8_t opcode;
static int offset;

void EXTI4_15_IRQHandler(void)
{
    EXTI->PR |= EXTI_PR_PR4;
    if ((GPIOA->IDR & GPIO_IDR_4)==0)
    {
        SPI1->CR1 &= ((uint16_t)0xFEFF); // Software SS low
        offset = -1;
    }
    else 
    { 
        while ((SPI1->SR & SPI_SR_RXNE) != 0) opcode = SPI_READ();
        SPI1->CR1 |= SPI_CR1_SSI;       // Software SS high
    }
}

void spi_slave_reset(void)
{
    NVIC_DisableIRQ(EXTI4_15_IRQn);
    NVIC_DisableIRQ(SPI1_IRQn);
    // enter reset
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    // leave reset
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
}

/**
 *   * @brief  This function handles SPI1 interrupt request.
 *   * @param  None
 *   * @retval None
 *   */

void SPI1_IRQHandler(void)
{
    uint8_t param;

    static void *jump_table[16] = {
        &&op_null,              // 0
        &&op_get_udid,          // 1
        &&op_get_signature,     // 2
        &&op_led,               // 3
        &&op_reset,             // 4
        &&op_get_error,         // 5
        &&op_get_status,        // 6
        &&op_store_data,        // 7
        &&op_store_send,        // 8
        &&op_fetch_data,        // 9
        &&op_fetch_ack,         // 10
        &&op_filter_write,      // 11
        &&op_filter_commit,     // 12
        &&op_filter_read,       // 13
        &&op_node_id,           // 14
        &&op_null,              // 15
    };
    static void *start_jump_table[16] = {
        &&op_start_null,            // 0
        &&op_start_get_udid,        // 1
        &&op_start_get_signature,   // 2
        &&op_start_led,             // 3
        &&op_start_reset,           // 4
        &&op_start_get_error,       // 5
        &&op_start_get_status,      // 6
        &&op_start_store_data,      // 7
        &&op_start_store_send,      // 8
        &&op_start_fetch_data,      // 9
        &&op_start_fetch_ack,       // 10
        &&op_start_filter_write,    // 11
        &&op_start_filter_commit,   // 12
        &&op_start_filter_read,     // 13
        &&op_start_node_id,         // 14
        &&op_start_null,            // 15
    }; 
    

    if ((SPI1->SR & SPI_SR_RXNE) == SPI_SR_RXNE)
    {
        if (offset < 0)
        {
            opcode = SPI_READ();
            offset = 0;
            goto *start_jump_table[opcode];
        }
        else 
        {
            param = SPI_READ();
            offset++; 
            goto *jump_table[opcode];
        }
    }
    return;


    /********** 0 **********/
op_start_null:      
op_null:            
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 1 **********/
op_start_get_udid:
op_get_udid:
    if (offset<8)
        SPI_WRITE(NOCAN_REGS.UDID[offset]);
    else
        SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 2 **********/
op_start_get_signature:
op_get_signature:
    if (offset<sizeof(NOCAN_REGS))
        SPI_WRITE(NOCAN_REGS.signature[offset]);
    else
        SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 3 **********/
op_start_led:
    SPI_WRITE(SPI_MORE_BYTE);
    return;
op_led:
    if (offset==1) {
        if (param==0) {
            SPI_WRITE(SPI_OK_BYTE);
            gpio_clear_led();
            NOCAN_STATUS_SET(NOCAN_STATUS_LED);
        } else {
            SPI_WRITE(SPI_OK_BYTE);
            gpio_set_led();
            NOCAN_STATUS_CLEAR(NOCAN_STATUS_LED);
        }
        return; 
    }
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 4 **********/
op_start_reset:   
    SPI_WRITE(SPI_MORE_BYTE);
    return;
op_reset:
    if (offset==1)
    {
        if (param==0)
        {
            SPI_WRITE(SPI_OK_BYTE);
            nocan_close();
            nocan_open();
            return;
        }
        if (param==1)
        {
            SPI_WRITE(SPI_OK_BYTE);
            NVIC_SystemReset();
            return;
        }
    } 
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 5 **********/
op_start_get_error:
    SPI_WRITE(NOCAN_REGS.STATUS);
    nocan_error_clear();
    return;
op_get_error:
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 6 **********/
op_start_get_status:
op_get_status:
    SPI_WRITE(NOCAN_REGS.STATUS);
    return;

    /********** 7 **********/
op_start_store_data: 
    SPI_WRITE(SPI_MORE_BYTE);
    return;
op_store_data:
    if (offset<CAN_BUFFER_LEN) {
        SPI_WRITE(SPI_OK_BYTE);
        can_tx_buffer[offset-1] = param;
        return;
    }
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 8 **********/
op_start_store_send:
    SPI_WRITE(SPI_OK_BYTE);
    gpio_clear_tx_int();            // pre-emptively signal transmition in progress
    can_commit_tx_buffer();
    //debug_push(DEVENT_MARK_1, can_tx_buffer[2]);
    return;
op_store_send:
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 9 **********/
op_start_fetch_data:
op_fetch_data:
    if (offset<CAN_BUFFER_LEN) {
        SPI_WRITE(can_rx_buffer[offset]);
        return;
    }
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 10 **********/
op_start_fetch_ack:
    SPI_WRITE(SPI_OK_BYTE);
    gpio_set_rx_int();            // pre-emptively signal receive buffer empty
    can_shift_rx_buffer();
    return;
op_fetch_ack:
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 11 **********/
op_start_filter_write:
    SPI_WRITE(SPI_MORE_BYTE);
    return; 
op_filter_write:
    if (offset==1) {
        SPI_WRITE(SPI_MORE_BYTE);
        offset = 3*param+2;
        return;
    }
    else if (offset<3*CAN_FILTER_COUNT+3) 
    {
        SPI_WRITE(SPI_OK_BYTE);
        can_filter_buffer[offset-3]=param;
        return;
    }
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 12 **********/
op_start_filter_commit:
    SPI_WRITE(SPI_MORE_BYTE);
    return;
op_filter_commit:
    if (offset==1) {
        SPI_WRITE(SPI_OK_BYTE);
        can_commit_filter(param);
        return;
    }
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 13 **********/
op_start_filter_read:
    SPI_WRITE(SPI_MORE_BYTE);
    return;
op_filter_read:
     if (offset==1) {
        SPI_WRITE(SPI_MORE_BYTE);
        offset = 3*param+2;
        return;
    } 
    if (offset<3*CAN_FILTER_COUNT+2)  
        SPI_WRITE(can_filter_buffer[offset-2]);
    else
        SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 14 **********/
op_start_node_id:
    SPI_WRITE(SPI_MORE_BYTE);
    return;
op_node_id:
    if (offset==1) {
        SPI_WRITE(SPI_OK_BYTE);
        can_node_filter(param);
    }
    return;

    /********** 15 **********/
    /* Reserved *************/
}

int spi_slave_init()
{
    /* Enable the peripheral clock of GPIOA */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* Select AF mode (10) on PA5, PA6, PA7 */
    GPIOA->MODER = (GPIOA->MODER 
            & ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7))
        | (GPIO_MODER_MODER5_1| GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);

    /* AF0 for SPI1 signals */
    GPIOA->AFR[1] = (GPIOA->AFR[1] &
            ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7)); 


    /* Enable input for GPIO PA4 */
    // Nothing to do since default state
    GPIOA->MODER &=  ~(GPIO_MODER_MODER4); 


    /* Enable the peripheral clock SPI1 */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* Configure SPI1 in slave */
    /* nSS hard, slave, CPOL and CPHA at zero (rising first edge) */
    /* (1) RXNE IT, 8-bit Rx fifo */
    /* (2) Enable SPI1 */
    SPI1->CR2 = SPI_CR2_RXNEIE                          // Enable RX buffer not empty interrupt
        | SPI_CR2_FRXTH                                 // RXNE event generated if FIFO level = 8 bits
        | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0    // DataSize=8 bits
        ; /* (1) */
    SPI1->CR1 |= (SPI_CR1_SPE                            // SPI enable
        | SPI_CR1_SSM)                                  // Software Slave Select
        ; /* (2) */

    /* Configure IT */
    /* (3) Set priority for SPI1_IRQn */
    /* (4) Enable SPI1_IRQn */
    NVIC_SetPriority(SPI1_IRQn, 0); /* (3) */
    NVIC_EnableIRQ(SPI1_IRQn); /* (4) */

    spi_slave_configure_EXTI();
    return 0;
}

