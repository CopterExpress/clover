#!/bin/bash

set -e

echo -e "\033[0;31m\033[1m$(date) | #1 Write to /etc/wpa_supplicant/wpa_supplicant.conf\033[0m\033[0m"

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

echo -e "\033[0;31m\033[1m$(date) | #2 Write STATIC to /etc/dhcpcd.conf\033[0m\033[0m"

echo "
interface wlan0
static ip_address=192.168.11.1/24" >> /etc/dhcpcd.conf

echo -e "\033[0;31m\033[1m$(date) | #3 Write dhcp-config to /etc/dnsmasq.conf\033[0m\033[0m"

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

echo -e "\033[0;31m\033[1m$(date) | #4 Write magic script for rename SSID to /etc/rc.local\033[0m\033[0m"

RENAME_SSID="sudo sed -i.OLD \"s/CLEVER/CLEVER-\$(head -c 100 /dev/urandom | xxd -ps -c 100 | sed -e 's/[^0-9]//g' | cut -c 1-4)/g\" /etc/wpa_supplicant/wpa_supplicant.conf && sudo sed -i '/sudo sed/d' /etc/rc.local && sudo reboot"

sed -i "19a$RENAME_SSID" /etc/rc.local

echo -e "\033[0;31m\033[1m$(date) | #5 End of network installation\033[0m\033[0m"
