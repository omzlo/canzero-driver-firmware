# CANZERO core driver for STM32F042

This repository contains all the source code for the STM32F042 ARM Cortex
MCU on the [CANZERO][1], which acts as a CAN bus driver.

## Directories

`driver` contains the code for the STM32F0 CAN bus driver.

## Compiling

Compilation relies on the the `arm-none-eabi-gcc` toolchain and `make`.

The approach is derived from the one described by [Geoffrey Brown, of University of Indiana][2]

Compiling the firmware requires the very small subset of CMSIS libraries that come with [stm32 standard peripheral library][3]. The code itself does not use standard peripheral library but relies on direct programming of STM32F0 registers.

To compile the code, you will need to edit the `LIBROOT` variable in the Makefile to point to the location of your copy of the standard peripheral library.

## License

This code is licensed under the MIT licence, as described in the LICENSE file.
[1]: https://omzlo.com/canzero
[1]: https://www.cs.indiana.edu/~geobrown/book.pdf
[2]: https://www.st.com/en/embedded-software/stm32-standard-peripheral-libraries.html?querycriteria=productId=LN1939
