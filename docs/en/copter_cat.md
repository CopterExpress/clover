# CopterCat_cm4

[CopterHack-2022](copterhack2022.md), команда **CopterCat**.

<img src="../assets/copter_cat/logo.svg" width="500">

## Team Information

Line-up:

* Lapin Matvey (https://t.me/l_motya), engineer/programmer.
* Konovalov Evgeny (https://t.me/egnknvlv), engineer/friend.
* Skandakov Egor (https://t.me/hjbaa), friend.
* Jalilov Emil, friend.

## Project Description

Development of a modern board for PX4 FMUv6U firmware, 55x40 mm sizes and an additional WiFi module to implement cool things, for example, a distributed network.

### Project idea

Flight controller on stm32h7 with space for RPi CM4 and built-in ESP32 to create a distributed network.

### Planned results

FMUv6U flight controller board and API for interacting with a distributed network via RPi.

### Using the "Clover" platform

At the stage of the project: debugging and demonstration of capabilities. After: using CopterCat as the main one.

## Specification

### FMU

* STM32H753IIK6 480Mhz Cortex-M7
* 2Mb + 256Kb FLASH
* 1Mb RAM
* ICM20602, ICM42605, BMI088, BMP388, BMM150
* Fully compliant with FMU-v6u standard

### Raspberry Pi

* Support for RPi CM4 board.
* SD card slot.
* Ability to flash the built-in eMMC.
* CAT24C256 EEPROM.
* Support 2 cameras (CAM0 is two lines, CAM1 is four lines).
* USB-OTG support.

### ESP32

* 16MB external FLASH (W25Q128JVS).
* 8MB external PSRAM (LY68L6400SLIT).
* Built-in antenna.
* USB-TTL converter.

### Remaining

* USB-HUB USB2514B.
* USB-PD with physical switching.
* Communication between ESP32 and STM32 via UART.
* 3 power options.
* 4 universal GPIOs from ESP32.
* USB Type-C.
* Dimensions 40x55mm, board 4 layers.

## Connectors and jumpers

![](../assets/copter_cat/board_top_nums.png)
![](../assets/copter_cat/board_bottom_nums.png)

1. GPIO ESP32 4 I/O ports for connecting external equipment.
2. RPi CM4 connectors.
3. ESC pins 8 pcs.
4. Programming and debugging pins for JTAG STM32.
5. Camera connectors (22 pin cable with 0.5 mm distance between conductors).
6. Contact for connecting the address tape.
7. Main power contacts 5V.
8. JST-6 standard PX4 power cable.
9. JST-6 GPS+compass+5V.
10. JST-4 I2C+5В.
11. USB Type-C.
12. JST-4 UART7+5В.
13. JST-4 I2C RPi+3.3B for connecting a rangefinder.
14. JST-4 UART5+5V.
15. JST-5 Standard connector for connecting the control receiver.
16. SD card slot (for RPi).
17. Jumper BOOT for STM32.
18. RPIBOOT jumper for flashing the RPi CM4 eMMC module.
19. Jumper for switching the USB connector operation mode (when the jumper is closed, USB works as a HUB input and when connected to a computer, STM32, ESP32 and RPi CM4 in OTG mode will be displayed; when the jumper is open, USB will work to connect external devices to the RPi, for example stereo cameras).

## Firmware download

### FMU

When you first start, the microcontroller will have to load the PX4-bootloader through the JTAG port. detailed instructions [here](https://docs.px4.io/master/en/software_update/stm32_bootloader.html#stm32-bootloader).

To connect to a computer:

1. Close jumper 19.
2. Connect USB Type-C to your computer.
3. The device should appear in [QGC](http://qgroundcontrol.com).

You can also flash firmware via RPi:

1. Install the RPi CM4 into the connector on the board.
2. Open jumper 19.
3. The device will appear in the `/dev` folder on the RPi.

### ESP32

You can write a program either in [Arduino IDE](https://www.arduino.cc/en/software) or in [VS Code](https://code.visualstudio.com) with the plugin [esp-idf](https://habr.com/ru/post/530638/). Then compile and upload to the microcontroller. You can download in two ways.

From computer:

1. Close jumper 19
2. CP2104 will appear in the connected devices
3. Download the firmware according to the instructions for the selected IDE

With RPi CM4:

1. Install the RPi CM4 into the connector on the board.
2. Open jumper 19.
3. Compile your code to .bin format.
4. Upload the resulting file to the RPi.
5. Download the firmware to the microcontroller using esptool.py [description+installation](https://docs.espressif.com/projects/esptool/en/latest/esp32/index.html).

### RPi CM4

The SD card slot works like a standard RPi. For boards with eMMC, the boot order of the operating system does not differ from the CM4 IO Board [instruction](https://www.jeffgeerling.com/blog/2020/how-flash-raspberry-pi-os-compute-module-4-emmc-usbboot).

## General information

* All files required for ordering are located in the `/gerbers` folder [here](https://github.com/matveylapin/CopterCat_cm4).
* The project was made in the program [KiCAD v6](https://www.kicad.org).
* Component Libraries are from [snapeda](https://www.snapeda.com).
