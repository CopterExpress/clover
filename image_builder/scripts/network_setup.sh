#! /usr/bin/env bash

#
# Script for image configure
# @urpylka Artem Smirnov
#

# Exit immidiately on non-zero result
set -e

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

  TEXT="$(date) | $1"
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
echo "
network={
    ssid=\"CLEVER\"
    psk=\"cleverwifi\"
    mode=2
    proto=RSN
    key_mgmt=WPA-PSK
    pairwise=CCMP
    group=CCMP
    auth_alg=OPEN
}" >> /etc/wpa_supplicant/wpa_supplicant.conf

echo_stamp "#2 Write STATIC to /etc/dhcpcd.conf"

echo "
interface wlan0
static ip_address=192.168.11.1/24" >> /etc/dhcpcd.conf

echo_stamp "#3 Write dhcp-config to /etc/dnsmasq.conf"

echo "
interface=wlan0
address=/clever/coex/192.168.11.1
dhcp-range=192.168.11.100,192.168.11.200,12h
no-hosts
filterwin2k
bogus-priv
domain-needed
quiet-dhcp6
" >> /etc/dnsmasq.conf

#echo_stamp "#4 Write magic script for rename SSID to /etc/rc.local"
#RENAME_SSID="sudo sed -i.OLD \"s/CLEVER/CLEVER-\$(head -c 100 /dev/urandom | xxd -ps -c 100 | sed -e 's/[^0-9]//g' | cut -c 1-4)/g\" /etc/wpa_supplicant/wpa_supplicant.conf && sudo sed -i '/sudo sed/d' /etc/rc.local && sudo reboot"
#sed -i "19a$RENAME_SSID" /etc/rc.local

echo_stamp "#5 End of network installation"
