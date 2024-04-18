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

NEW_SSID='clover-'$(head -c 100 /dev/urandom | xxd -ps -c 100 | sed -e "s/[^0-9]//g" | cut -c 1-4)
echo "--- Setting SSID to ${NEW_SSID}"
# TODO: Use wpa_cli insted direct file edit
# FIXME: We rely on raspberrypi-net-mods to copy our file to /etc/wpa_supplicant.
# This is not very reliable, but seems to fix our rfkill problem.
cat << EOF >> /boot/wpa_supplicant.conf
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=GB
network={
    ssid="${NEW_SSID}"
    psk="cloverwifi"
    mode=2
    proto=WPA RSN
    key_mgmt=WPA-PSK
    pairwise=CCMP
    group=CCMP
    auth_alg=OPEN
}
EOF

NEW_HOSTNAME=$(echo ${NEW_SSID} | tr '[:upper:]' '[:lower:]')
echo "--- Setting hostname to $NEW_HOSTNAME"
hostnamectl set-hostname $NEW_HOSTNAME
sed -i 's/127\.0\.1\.1.*/127.0.1.1\t'${NEW_HOSTNAME}' '${NEW_HOSTNAME}'.local/g' /etc/hosts
# .local (mdns) hostname added to make it accesable when wlan and ethernet interfaces are down

echo "--- Enable ROS services"
systemctl enable roscore
systemctl enable clover

echo "--- Harware setup"
/root/hardware_setup.sh

echo "--- Remove init scripts"
rm /root/init_rpi.sh /root/hardware_setup.sh
