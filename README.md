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

Currently only the software based rendering is supported in a single thread mode, in the current optimization
level, it is possible to execute the widgets demo with a value between 25 and 30FPS, while complex container
rendering from the benchmark demo slowdowns to around 10FPS. There are plans to enable the Hardware aided
rendering based on the VGlite compatible GPU for the future releases.

## Specification

### CPU and Memory
- **MCU:** NXP IMXRT1176 Dual ARM Core Cortex-M7 + Cortex M4
- **RAM:** 1MB of internal scattered RAM, 32MB of external SDRAM
- **Flash:** 16MB of QSPI based External Flash
- **GPU:** VGLite compatible GPU

### Display and Touch
- **Resolution:** 480 x 800
- **Display Size:** 7"
- **Interface:** MIPI-DSI
- **Color Depth:** 32-bit ARGB8888 format
- **Technology:** IPS
- **Touch Pad:** Capacitive touch panel

### Connectivity
- 2x Ethernet
- 2x CAN Bus
- Serial peripherals: UART/SPI/I2C
- SD Card based on SDHC
- Audio IN + OUT
- Arduino R1 connector standard

## Getting started

### Hardware setup
- First of all disconnect all power cables from the board
- Connect the display panel to the board FPC-type connector J48
- Connect USB cable to the J86 micro-USB it provides both Debug port plus console
- Finally connect the power cable to the board barrel-jack DC_5V_IN

### Software setup
- You need to install the MCU-Expresso IDE from NXP website:
    * It is avaibalble to Linux, Windows or Mac
    * Refer the link: https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE


### Run the project
- Clone this repository repository
- Open the NXP MCU Expresso IDE:
    * Go to the toolbars and enter in the File menu
    * Then **Import->Existing Projects in the workspace**
    * Navigate to the path where you cloned this repo
    * Then hit OK
- Build the project:
    * On the project explorer do a right click on the imported project
    * Click on **Build Project**
- Run or Debug:
    * First build the project as instructed above
    * On the project explorer do a right click on the imported project
    * Click on the **Debug As...**
    * Select you debug probe configuration it can be:
        * MCU Expresso Link server
        * Jlink Probe
    * Select the discovered probe and then hit Debug
    * The firmware will be downloaded and the debug will be halted on the **main()**
    * just hit **F8** to resume the debuggin
### Debugging
- You can also complement the debug experience with the printf / Console:
    * With the board connected to the host computer find its **COM** number or **/dev/<serial>** node
    * Open your favorite terminal program;
    * Connnect to the serial port found with the following settings:
        * 8 bits
        * 115200 bps
        * No parity
        * No flow control
    * You should now see the logs reported by the demo application while it runs

## Known limitations:
The code area of this application being linked to the QSPI external flash, which does not extract
the full execution speed of this demo, but brings the simplicity to the getting started steps, the user
may change it during experimentation under MCU settings option. Also currently, only the SW render unit
is enabled, the support for hardware acceleration based on VGLite and PXP will be added soon.

## Contribution and Support

If you find any issues with the development board feel free to open an Issue in this repository. For LVGL related issues (features, bugs, etc) please use the main [lvgl repository](https://github.com/lvgl/lvgl).

If you found a bug and found a solution too please send a Pull request. If you are new to Pull requests refer to [Our Guide](https://docs.lvgl.io/master/CONTRIBUTING.html#pull-request) to learn the basics.

