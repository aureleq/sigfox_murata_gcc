# ST Eval Kit Murata Sigfox - GCC version

This project provides a Makefile to compile ST Sigfox SDK to use with Murata eval board B-L072Z-LRWAN1.
It contains the full arborescence of [X-CUBE-SFOX](https://www.st.com/content/st_com/en/products/embedded-software/mcus-embedded-software/stm32-embedded-software/stm32cube-expansion-packages/x-cube-sfox.html) (v1.1.0), only the Makefile was added in order to compile with ARM-gcc and flash with the firmware with ST-flash.

it is an alternative to IDE-based examples that allows you to use your preferred code editor and command line tools.

## Requirements

The [GNU Embedded Toolchain for Arm](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) must be installed to cross-compile the example.

The tool [ST-flash](https://github.com/texane/stlink) is used to load the firmware, you can install this tool or use the traditional STM32 ST Link Utility.

For Windows users, additional GNU utilities maybe required, especially to interprete the Makefile.
[MSYS](http://www.mingw.org/wiki/msys) is recommended for this usage.

Don't forget to update your PATH variable for all those tools.

If you haven't used your board with Sigfox yet, you will need a bit of soldering first, you can refer to [this other tutorial](https://github.com/aureleq/muRataSigfox). You can also retrieve your Sigfox IDs and Keys following this link.


## Usage

Makefile is provided for the Push Button example only, it can easily be adapted for the ModemAT example.

if you wish to use your Sigfox IDs, your will need to copy first your `sigfox_data.h` file in the following directory: *//Projects/B-L072Z-LRWAN1/Applications/Sgfx/Sgfx_push_button/inc*

To compile the code:
```
make all
```

Flashing the device requires the device to be in bootloader mode. To do so on the eval board, press down the Reset Button while connecting USB cable to your computer (you will see the LED flashing), then release it.
```
make flash
```
