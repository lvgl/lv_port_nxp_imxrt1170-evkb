# LVGL ported to IMXRT-1170-EVKB

## Overview

This port is intended to use with the IMXRT-1170-EVK with B revision, please use only the B revision since
the previous revision has a different flash configuration. You will also need the `RK055HDMIPI4MA0` display board,
other displays may be plugged but code changes on the touch panel and display settings may be required.

The idea of this project is to provide a quick start to users that have this board to deploy they first
LVGL application to it, by default this project execute the `lvgl_demo_widgets` where a collection of
widgets get drawn on the display allowing the user to interact to it.

This project is itended to be used with MCU Expresso IDE.

## Buy

The board combo can be acquired from the NXP Direct channel:

* https://www.nxp.com/design/design-center/development-boards-and-designs/i-mx-evaluation-and-development-boards/i-mx-rt1170-evaluation-kit:MIMXRT1170-EVKB

Also its also obtained via Mouser:

* https://br.mouser.com/ProductDetail/NXP-Semiconductors/MIMXRT1170-EVKB?qs=Jm2GQyTW%2Fbj7SYhf%2F1gIQA%3D%3D


## Benchmark

Describe the default buffering and other configuration.

YouTube video link

## Specification

### CPU and Memory
- **MCU:**
- **RAM:** ...MB internal, ...MB external SDRAM
- **Flash:** ...MB internal, ..MB External
- **GPU:** if any

### Display and Touch
- **Resolution:** ...x...
- **Display Size:** ..."
- **Interface:** SPI/LCD/MIPI/etc
- **Color Depth:** ...-bit
- **Technology:** TN/IPS
- **DPI:** ... px/inch
- **Touch Pad:** Resistive/Capacitive/None

### Connectivity
- Other peripheries

## Getting started

### Hardware setup
- jumpers, switches
- connect the display
- which USB port to use

### Software setup
- Install drivers if needed
- Install the IDE + links

### Run the project
- Clone this repository repository: ...
- Open the terminal or Import into an IDE...
- Build the project. How?
- Run or Debug. How?

### Debugging
- Debug  `printf`?
- Other?
-
## Notes

Other notes, e.g. different configs, optimization opportunities, adding other libraries to the project, etc

## Contribution and Support

If you find any issues with the development board feel free to open an Issue in this repository. For LVGL related issues (features, bugs, etc) please use the main [lvgl repository](https://github.com/lvgl/lvgl).

If you found a bug and found a solution too please send a Pull request. If you are new to Pull requests refer to [Our Guide](https://docs.lvgl.io/master/CONTRIBUTING.html#pull-request) to learn the basics.

