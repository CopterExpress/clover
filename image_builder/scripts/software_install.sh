#!/bin/bash

set -e

##################################################################################################################################
# Image software installation
##################################################################################################################################

echo -e "\033[0;31m\033[1m$(date) | #1 Software installing\033[0m\033[0m"

# TODO: Use dnsmasq instead of isc-dhcp-server
apt-get install --no-install-recommends -y \
  unzip=6.0-21 \
  zip=3.0-11 \
  ipython=5.1.0-3 \
  ipython3=5.1.0-3 \
  screen=4.5.0-6 \
  byobu=5.112-1  \
  nmap=7.40-1 \
  lsof=4.89+dfsg-0.1 \
  git=1:2.11.0-3+deb9u3 \
  dnsmasq=2.76-5+rpt1+deb9u1  \
  tmux=2.3-4 \
  vim=2:8.0.0197-4+deb9u1 \
  cmake=3.7.2-1 \
  python-pip=9.0.1-2+rpt2 \
  python3-pip=9.0.1-2+rpt2 \
  libjpeg8-dev=8d1-2 \
  tcpdump \
  libpoco-dev
  
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
