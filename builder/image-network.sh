#! /usr/bin/env bash

#
# Script for network configuration
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

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

  TEXT="$(date '+[%Y-%m-%d %H:%M:%S]') $1"
  TEXT="\e[1m$TEXT\e[0m" # BOLD

  case "$2" in
    SUCCESS)
    TEXT="\e[32m${TEXT}\e[0m";; # GREEN
    ERROR)
    TEXT="\e[31m${TEXT}\e[0m";; # RED
    *)
    TEXT="\e[34m${TEXT}\e[0m";; # BLUE
  esac
  echo -e ${TEXT}
}

echo_stamp "#1 Write STATIC to /etc/dhcpcd.conf"

cat << EOF >> /etc/dhcpcd.conf
interface wlan0
static ip_address=192.168.11.1/24
EOF

echo_stamp "#2 Set wpa_supplicant country"

cat << EOF >> /etc/wpa_supplicant/wpa_supplicant.conf
country=GB
EOF

echo_stamp "#3 Write dhcp-config to /etc/dnsmasq.conf"

cat << EOF >> /etc/dnsmasq.conf
interface=wlan0
address=/clover/coex/192.168.11.1
dhcp-range=192.168.11.100,192.168.11.200,12h
no-hosts
filterwin2k
bogus-priv
domain-needed
quiet-dhcp6
EOF

echo_stamp "#4 Build the RTL8814AU Wi-Fi adapter driver"
cd /home/pi
git clone https://github.com/aircrack-ng/rtl8812au.git --depth=1
cd rtl8812au
echo kernel version: $(uname -r)
echo kernel version from procfs: $(cat /proc/version)
echo version: $(git describe --tags --always)
sed -i 's/CONFIG_PLATFORM_I386_PC = y/CONFIG_PLATFORM_I386_PC = n/g' Makefile # https://github.com/aircrack-ng/rtl8812au#for-raspberry-rpi
sed -i 's/CONFIG_PLATFORM_ARM_RPI = n/CONFIG_PLATFORM_ARM_RPI = y/g' Makefile
# sed -i 's/CONFIG_PLATFORM_ARM64_RPI = n/CONFIG_PLATFORM_ARM64_RPI = y/g' Makefile
apt-cache policy raspberrypi-kernel-headers
apt-get install -y raspberrypi-kernel-headers dkms
ls /lib/modules
KERNEL_VERSION=5.10.17-v7l+ # TODO: get kernel version from the fs
echo make
make KERNEL_VER=$KERNEL_VERSION KVER=$KERNEL_VERSION # TODO: determine kernel version from fs
echo make install
make install KERNEL_VER=$KERNEL_VERSION KVER=$KERNEL_VERSION

echo_stamp "#5 End of network installation"
