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

set -ex # exit on error, echo commands

echo "--- Write static to /etc/dhcpcd.conf"
cat << EOF >> /etc/dhcpcd.conf
interface wlan0
static ip_address=192.168.11.1/24
EOF

echo "--- Set wpa_supplicant country"
cat << EOF >> /etc/wpa_supplicant/wpa_supplicant.conf
country=GB
EOF

echo "--- Write dhcp-config to /etc/dnsmasq.conf"
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
