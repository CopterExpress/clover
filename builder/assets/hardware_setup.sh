#! /usr/bin/env bash

#
# Script for build the image. Used builder script of the target repo
# For build: docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/builder/repo smirart/builder
#
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Artem Smirnov <urpylka@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -e # Exit immidiately on non-zero result

##################################################
# Configure hardware interfaces
##################################################

# 1. Enable sshd
echo "--- Enable sshd"
touch /boot/ssh
# /usr/bin/raspi-config nonint do_ssh 0

# 2. Enable GPIO
echo "--- GPIO enabled by default"

# 3. Enable I2C
echo "--- Enable I2C"
/usr/bin/raspi-config nonint do_i2c 0

# 4. Enable SPI
echo "--- Enable SPI"
/usr/bin/raspi-config nonint do_spi 0

# 5. Enable raspicam
echo  "--- Enable raspicam"
/usr/bin/raspi-config nonint do_camera 0

# 6. Enable hardware UART
echo "--- Enable UART"
# Temporary solution
# https://github.com/RPi-Distro/raspi-config/pull/75
/usr/bin/raspi-config nonint do_serial 1
/usr/bin/raspi-config nonint set_config_var enable_uart 1 /boot/config.txt
echo dtoverlay=pi3-disable-bt >> /boot/config.txt
systemctl disable hciuart.service

# After adding to Raspbian OS
# https://github.com/RPi-Distro/raspi-config/commit/d6d9ecc0d9cbe4aaa9744ae733b9cb239e79c116
#/usr/bin/raspi-config nonint do_serial 2

# 7. Enable V4L driver http://robocraft.ru/blog/electronics/3158.html
#echo "bcm2835-v4l2" >> /etc/modules
echo "--- Enable v4l2 driver"
if ! grep -q "^bcm2835-v4l2" /etc/modules;
then printf "bcm2835-v4l2\n" >> /etc/modules
fi
