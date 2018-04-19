#ifndef _GPIO_H_
#define _GPIO_H_

#define HIGH 1
#define LOW  0

#define CAN_RX_INT_Pin  0    
#define CAN_TX_INT_Pin  1
#define CAN_LED_Pin     1
#define RESET_Pin       8


void gpio_init(void);

#define gpio_set_tx_int() (GPIOA->BSRR = (1<<CAN_TX_INT_Pin))
#define gpio_clear_tx_int() (GPIOA->BRR = (1<<CAN_TX_INT_Pin))

#define gpio_set_rx_int() (GPIOA->BSRR = (1<<CAN_RX_INT_Pin))
#define gpio_clear_rx_int() (GPIOA->BRR = (1<<CAN_RX_INT_Pin))

#define gpio_set_led() (GPIOB->BSRR = (1<<CAN_LED_Pin))
#define gpio_clear_led() (GPIOB->BRR = (1<<CAN_LED_Pin))

#define gpio_set_reset() (GPIOB->BSRR = (1<<RESET_Pin))
#define gpio_clear_reset() (GPIOB->BRR = (1<<RESET_Pin))


#endif
