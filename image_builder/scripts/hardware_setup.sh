#!/bin/bash

# Exit immidiately on non-zero result
set -e

#
# Script for image configure
# @urpylka Artem Smirnov
#

##################################################
# Configure hardware interfaces
##################################################

# 1. Enable sshd
echo -e "\033[0;31m\033[1m$(date) | #1 Turn on sshd\033[0m\033[0m"
touch /boot/ssh
# /usr/bin/raspi-config nonint do_ssh 0

# 2. Enable GPIO
echo -e "\033[0;31m\033[1m$(date) | #2 GPIO enabled by default\033[0m\033[0m"

# 3. Enable I2C
echo -e "\033[0;31m\033[1m$(date) | #3 Turn on I2C\033[0m\033[0m"
/usr/bin/raspi-config nonint do_i2c 0

# 4. Enable SPI
echo -e "\033[0;31m\033[1m$(date) | #4 Turn on SPI\033[0m\033[0m"
/usr/bin/raspi-config nonint do_spi 0

# 5. Enable raspicam
echo -e "\033[0;31m\033[1m$(date) | #5 Turn on raspicam\033[0m\033[0m"
/usr/bin/raspi-config nonint do_camera 0

# 6. Enable hardware UART
echo -e "\033[0;31m\033[1m$(date) | #6 Turn on UART\033[0m\033[0m"
# Temporary solution
# https://github.com/RPi-Distro/raspi-config/pull/75
/usr/bin/raspi-config nonint do_serial 1
/usr/bin/raspi-config nonint set_config_var enable_uart 1 /boot/config.txt

# After adding to Raspbian OS
# https://github.com/RPi-Distro/raspi-config/commit/d6d9ecc0d9cbe4aaa9744ae733b9cb239e79c116
#/usr/bin/raspi-config nonint do_serial 2

# 7. Enable V4L driver http://robocraft.ru/blog/electronics/3158.html
#echo "bcm2835-v4l2" >> /etc/modules
echo -e "\033[0;31m\033[1m$(date) | #7 Turn on v4l2 driver\033[0m\033[0m"
if ! grep -q "^bcm2835-v4l2" /etc/modules;
then printf "bcm2835-v4l2\n" >> /etc/modules
fi

echo -e "\033[0;31m\033[1m$(date) | End of configure hardware interfaces\033[0m\033[0m"
