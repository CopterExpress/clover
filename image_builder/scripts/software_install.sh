#!/bin/bash

set -e

##################################################################################################################################
# Image software installation
##################################################################################################################################

echo -e "\033[0;31m\033[1m$(date) | #1 Software installing\033[0m\033[0m"

# TODO: Use dnsmasq instead of isc-dhcp-server
apt-get install --no-install-recommends -y \
  unzip \
  zip \
  ipython \
  screen \
  byobu \
  nmap \
  lsof \
  git \
  dnsmasq \
  tmux \
  vim \
  ipython3 \
  cmake \
  python-pip \
  python3-pip \
  libjpeg8-dev
  
echo -e "\033[0;31m\033[1m$(date) | #2 Adding mjpg-streamer at /home/pi\033[0m\033[0m"
# https://github.com/jacksonliam/mjpg-streamer

git clone https://github.com/jacksonliam/mjpg-streamer.git /home/pi/mjpg-streamer \
  && cd /home/pi/mjpg-streamer/mjpg-streamer-experimental \
  && make \
  && make install \
  && chown -Rf pi:pi /home/pi/mjpg-streamer

echo -e "\033[0;31m\033[1m$(date) | Add .vimrc\033[0m\033[0m"

# vim settings
echo "set mouse-=a
syntax on
" > /home/pi/.vimrc

echo -e "\033[0;31m\033[1m$(date) | End of network installation\033[0m\033[0m"
