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

echo "--- Fix home directory permissions"
chmod +rx /home/pi

NEW_SSID='clover-'$(head -c 100 /dev/urandom | xxd -ps -c 100 | sed -e "s/[^0-9]//g" | cut -c 1-4)
echo "--- Creating Wi-Fi AP with SSID=${NEW_SSID}"
nmcli con add type wifi ifname wlan0 mode ap con-name clover ssid $NEW_SSID autoconnect true \
&& nmcli con modify clover 802-11-wireless.band bg \
&& nmcli con modify clover ipv4.method shared ipv4.address 192.168.11.1/24 \
&& nmcli con modify clover ipv6.method disabled \
&& nmcli con modify clover wifi-sec.key-mgmt wpa-psk \
&& nmcli con modify clover wifi-sec.psk "cloverwifi" \
&& systemctl disable dnsmasq # disable dnsmasq to avoid conflicts with NetworkManager's dnsmasq

NEW_HOSTNAME=$(echo ${NEW_SSID} | tr '[:upper:]' '[:lower:]')
echo "--- Setting hostname to $NEW_HOSTNAME"
hostnamectl set-hostname $NEW_HOSTNAME \
&& sed -i 's/127\.0\.1\.1.*/127.0.1.1\t'${NEW_HOSTNAME}' '${NEW_HOSTNAME}'.local/g' /etc/hosts
# .local (mdns) hostname added to make it accesable when wlan and ethernet interfaces are down

echo "--- Enable ROS services"
systemctl enable roscore
systemctl enable clover

echo "--- Harware setup"
/root/hardware_setup.sh

echo "--- Remove init scripts"
rm /root/init_rpi.sh /root/hardware_setup.sh
