#include <stm32f0xx.h>
#include "can.h"
#include "spi_slave.h"
#include "chip_options.h"
#include "systick.h"
#include "gpio.h"
#include "nocan.h"
#include "usart.h"
#include "debug_event.h"

int main(void)
{
  int n = 0;
  char c;

  debug_init();
  debug_push(DEVENT_START, 0x12345678);

  systick_init();
  
  usart_init(115200);
  
  gpio_init();

  gpio_set_led();
  
  chip_options_configure();

  gpio_clear_led();

  spi_slave_init();

  nocan_open();
 
  usart_printf("\nStarting serial console.\n");
  for(;;)
  {
      if (usart_available())
      {
          c = usart_getc();
          switch (c) {
              case 'b':
                  usart_printf("\nLed off");
                  gpio_clear_led();
                  break;
              case 'B':
                  usart_printf("\nLed on");
                  gpio_set_led();
                  break;
              case 'd': 
                  usart_printf("\nregisters: ");
                  for (int i=0;i<sizeof(NOCAN_REGS);i++)
                      usart_printf("%x ", ((uint8_t *)&NOCAN_REGS)[i]);
                  usart_printf("\nstat  : %x (%s)", NOCAN_REGS.STATUS, nocan_status_string());
                  usart_printf("\nver   : %x", NOCAN_REGS.VERSION);
                  usart_printf("\ntx/rx : %u/%u", NOCAN_REGS.CAN_TX_COUNT, NOCAN_REGS.CAN_RX_COUNT);
                  break;
              case 'f':
                  usart_printf("\nfilters: ");
                  for (uint32_t f=0;f<CAN_FILTER_COUNT;f++)
                  {
                      usart_printf("\n%x %x %x: ",
                              can_filter_buffer[3*f], 
                              can_filter_buffer[3*f+1], 
                              can_filter_buffer[3*f+2]);
                  }
                  break;
              case 'h': 
                  usart_printf("\ncommands: [b] set led off/on [c] send address request [f] dump can filters [h] help, [i] read chip uid, [d] dump registers, [p] 1 second systick pause [r] reset SAMD21 [z] reset stm32f0."); 
                  break;
              case 'i':
                  usart_printf("\nChip UID: ");
                  for (int i=0;i<12;i++) 
                      usart_printf("%x ", CHIP_UDID[i]);
                  break;
              case 'c':
                  usart_printf("\nconnecting: ");
                  if (can_sys_send(0, NOCAN_SYS_ADDRESS_REQUEST, 0, NOCAN_REGS.UDID, 8)<0) 
                      usart_printf("FAIL");
                  else
                      usart_printf("OK");
                  break;
              case 'p':
                  usart_printf("\npause: ");
                  systick_delay(1000);
                  usart_printf("OK");
                  break;
              case 'r':
                  gpio_clear_led();
                  usart_printf("\nreseting atmega328pb");
                  gpio_clear_reset();
                  for (uint32_t count=0;count<10000;count++) asm("nop");
                  gpio_set_reset();
                  break;
              case 'z':
                  usart_printf("\nreseting self (stm32f0)...");
                  NVIC_SystemReset();
                  break;
          }
          usart_printf("\n[h,b,B,c,d,f,i,q,r,z]?");
      }
      else
      {
          uint32_t h,l;
          if (debug_pop(&h,&l) == 0)
          {
            usart_printf("\n#%x #%x", h, l);
          }
      }
  }
}

/*
 * Debug help -- if asserts are enabled, assertion failure
 * from standard peripheral  library ends up here 
 */


#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* Infinite loop */
  /* Use GDB to find out why we're here */
  while (1)
  {
  }
}
#endif

