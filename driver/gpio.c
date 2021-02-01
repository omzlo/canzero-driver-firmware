#include <stm32f0xx.h>
#include "stm32f0_helpers.h"
#include "gpio.h"

/*
 * CAN_RX_INT: PA0
 * CAN_TX_INT: PA1
 * LED: PB1
 * RESET: PB8-BOOT0
 */

void gpio_init(void)
{
  // enable clock on GPIOA
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  // Configure pins on GPIOA
  GPIO_CONFIGURE_OUTPUT(GPIOA, CAN_TX_INT_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL);
  GPIO_CONFIGURE_OUTPUT(GPIOA, CAN_RX_INT_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL);
 
  // enable clock on GPIOB
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  // set it to high
  GPIOB->BSRR = (1<<RESET_Pin);

  // configure pins on GPIOB
  GPIO_CONFIGURE_OUTPUT(GPIOB, CAN_LED_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL);
  GPIO_CONFIGURE_OUTPUT(GPIOB, RESET_Pin, GPIO_SPEED_LOW, GPIO_OPEN_DRAIN);
  
}



