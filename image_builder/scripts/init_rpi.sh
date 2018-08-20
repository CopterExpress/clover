#!/bin/bash

echo_stamp() {

  # STATIC FUNCTION
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

echo_stamp "#1 Rename SSID"
sudo sed -i.OLD "s/CLEVER/CLEVER-$(head -c 100 /dev/urandom | xxd -ps -c 100 | sed -e 's/[^0-9]//g' | cut -c 1-4)/g" /etc/wpa_supplicant/wpa_supplicant.conf

echo_stamp "#2 Harware setup"
/root/hardware_setup.sh

echo_stamp "#3 Remove init scripts"
rm /root/init_rpi.sh /root/hardware_setup.sh

echo_stamp "#4 End of network installation"
