#!/bin/bash

echo -e "\033[0;31m\033[1m$(date) | #1 Rename SSID\033[0m\033[0m"
sudo sed -i.OLD "s/CLEVER/CLEVER-$(head -c 100 /dev/urandom | xxd -ps -c 100 | sed -e 's/[^0-9]//g' | cut -c 1-4)/g" /etc/wpa_supplicant/wpa_supplicant.conf

echo -e "\033[0;31m\033[1m$(date) | #2 Harware setup\033[0m\033[0m"
/root/hardware_setup.sh

echo -e "\033[0;31m\033[1m$(date) | #3 End of network installation\033[0m\033[0m"
