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

echo_stamp "#1 Write to /etc/wpa_supplicant/wpa_supplicant.conf"

# TODO: Use wpa_cli insted direct file edit
cat << EOF >> /etc/wpa_supplicant/wpa_supplicant.conf
network={
    ssid="clover"
    psk="cloverwifi"
    mode=2
    proto=RSN
    key_mgmt=WPA-PSK
    pairwise=CCMP
    group=CCMP
    auth_alg=OPEN
}
EOF

echo_stamp "#2 Write STATIC to /etc/dhcpcd.conf"

cat << EOF >> /etc/dhcpcd.conf
interface wlan0
static ip_address=192.168.11.1/24
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

echo_stamp "#5 End of network installation"
