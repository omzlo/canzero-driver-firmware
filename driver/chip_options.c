#include <stm32f0xx.h>
#include "usart.h"
#include "chip_options.h"
#include "gpio.h"
#include "systick.h"

const uint8_t *CHIP_UDID = (const uint8_t *)0x1FFFF7AC;


static void option_prog(volatile uint16_t* opt_addr, uint8_t data)
{
  /* (1) Set the PG bit in the FLASH_CR register to enable programming */
  /* (2) Perform the data write */
  /* (3) Wait until the BSY bit is reset in the FLASH_SR register */
  /* (4) Check the EOP flag in the FLASH_SR register */
  /* (5) Clear the EOP flag by software by writing it at 1 */
  /* (6) Reset the PG Bit to disable programming */
  FLASH->CR |= FLASH_CR_OPTPG; /* (1) */
  *opt_addr = data; /* (2) */
  while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (3) */
  {
    /* For robust implementation, add here time-out management */
  }  
  if ((FLASH->SR & FLASH_SR_EOP) != 0)  /* (4) */
  {
    FLASH->SR |= FLASH_SR_EOP; /* (5) */
    usart_printf("Success\n");
  }
  /* Manage the error cases */
  else if ((FLASH->SR & FLASH_SR_PGERR) != 0) /* Check programming error */
  {
    FLASH->SR |= FLASH_SR_PGERR; /* Clear the flag */
    usart_printf("ERROR_PROG_FLASH\n");
  }
  else if ((FLASH->SR & FLASH_SR_WRPERR) != 0)
  {
    FLASH->SR |= FLASH_SR_WRPERR; /* Clear the flag */
    usart_printf("ERROR_WRITE_PROTECTION\n");
  }     
  FLASH->CR &= ~FLASH_CR_OPTPG; /* (6) */
}

void chip_options_configure(void)
{
    if (OB->USER == 0xFF)
    {
        usart_printf("\nChanging OB->user\n");
        usart_printf("OB->USER = %x\n", OB->USER);

        // PREPARE FLASH OPERATION
        /* (1) Wait till no operation is on going */
        /* (2) Check that the Flash is unlocked */
        /* (3) Perform unlock sequence for Flash */
        /* (4) Check that the Option Bytes are unlocked */
        /* (5) Perform unlock sequence for Option Bytes */
        while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */  
        {
            /* For robust implementation, add here time-out management */
        }  
        if ((FLASH->CR & FLASH_CR_LOCK) != 0) /* (2) */
        {    
            FLASH->KEYR = FLASH_FKEY1; /* (3) */
            FLASH->KEYR = FLASH_FKEY2;
        }
        if ((FLASH->CR & FLASH_CR_OPTWRE) == 0) /* (4) */
        {
            FLASH->OPTKEYR = FLASH_OPTKEY1; /* (5) */
            FLASH->OPTKEYR = FLASH_OPTKEY2;
        }

        /* (1) Set the OPTER bit in the FLASH_CR register to enable option byte erasing */
        /* (2) Set the STRT bit in the FLASH_CR register to start the erasing */
        /* (3) Wait until the BSY bit is reset in the FLASH_SR register */
        /* (4) Check the EOP flag in the FLASH_SR register */
        /* (5) Clear EOP flag by software by writing EOP at 1 */
        /* (6) Reset the PER Bit to disable the page erase */
        FLASH->CR |= FLASH_CR_OPTER; /* (1) */
        FLASH->CR |= FLASH_CR_STRT; /* (2) */    
        while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (3) */
        {
            /* For robust implementation, add here time-out management */
        }
        if ((FLASH->SR & FLASH_SR_EOP) != 0)  /* (4) */
        {
            FLASH->SR |= FLASH_SR_EOP; /* (5)*/
        }    
        /* Manage the error cases */
        else if ((FLASH->SR & FLASH_SR_WRPERR) != 0) /* Check Write protection error error */
        {
            FLASH->SR |= FLASH_SR_WRPERR;    /* Clear the flag */
            usart_printf("ERROR_WRITE_PROTECTION\n");
        }
        else
        {
            usart_printf("ERROR_UNKNOWN\n");
        }
        FLASH->CR &= ~FLASH_CR_OPTER; /* (6) */


        // PROGRAM THE OPTION BYTE
        // Program the read protection to factory default
        option_prog(&(OB->RDP), (uint8_t)0xAA); 
        // set BOOT_SEL to 0, keep nBOOT0 to 1 (nBOOT1 is ignored), to set boot from flash 
        option_prog(&(OB->USER), (uint8_t)0x7F);
        usart_printf("Halting: A full power off must be conducted for the changes to take effect.\n");
        for(;;)
        {
            gpio_set_led();
            active_delay(1000);
            gpio_clear_led();
            active_delay(1000);

            gpio_set_led();
            active_delay(250);
            gpio_clear_led();
            active_delay(250);
            gpio_set_led();
            active_delay(250);
            gpio_clear_led();
            active_delay(250);             
        }
    }
}
