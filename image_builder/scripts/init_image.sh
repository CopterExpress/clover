#!/bin/bash

##################################################################################################################################
# Image initialisation
##################################################################################################################################

set -e

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

# Add apt key to allow local mirror usage during image build
#wget -O - ftp://192.168.0.10/coex-mirror.gpg | apt-key add -
# Generate a backup of the original source.list
#cp /etc/apt/sources.list /var/sources.list.bak
# Add the local mirror as the first priority repository
#wget -O - ftp://192.168.0.10/coex-mirror.list 2>/dev/null | cat - /etc/apt/sources.list > /var/sources.list && mv /var/sources.list /etc/apt/sources.list

echo_stamp "#1 apt cache update"

# Clean repostory cache
apt-get clean -qq > /dev/null
# Update repository cache
apt-get update -qq > /dev/null
# && apt upgrade -y

echo_stamp "#2 Write clever information"

# Clever image version
echo "$1" >> /etc/clever_version
# Origin image file name
echo "${2%.*}" >> /etc/clever_origin

echo_stamp "#3 Write magic script to /etc/rc.local"
MAGIC_SCRIPT="sudo /root/init_rpi.sh; sudo sed -i '/sudo \\\/root\\\/init_rpi.sh/d' /etc/rc.local && sudo reboot"
sed -i "19a${MAGIC_SCRIPT}" /etc/rc.local

echo_stamp "#4 End initialisation of image"
