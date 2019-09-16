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

NEW_SSID='clover-'$(head -c 100 /dev/urandom | xxd -ps -c 100 | sed -e "s/[^0-9]//g" | cut -c 1-4)
echo_stamp "Setting SSID to ${NEW_SSID}"
sudo sed -i.OLD "s/clover/${NEW_SSID}/" /etc/wpa_supplicant/wpa_supplicant.conf

NEW_HOSTNAME=$(echo ${NEW_SSID} | tr '[:upper:]' '[:lower:]')
echo_stamp "Setting hostname to $NEW_HOSTNAME"
hostnamectl set-hostname $NEW_HOSTNAME
sed -i 's/127\.0\.1\.1.*/127.0.1.1\t'${NEW_HOSTNAME}' '${NEW_HOSTNAME}'.local/g' /etc/hosts
# .local (mdns) hostname added to make it accesable when wlan and ethernet interfaces are down

echo_stamp "Harware setup"
/root/hardware_setup.sh

echo_stamp "Remove init scripts"
rm /root/init_rpi.sh /root/hardware_setup.sh

echo_stamp "End of initialization of the image"
