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
  ltrace \
  libpoco-dev=1.7.6+dfsg1-5+deb9u1

# Deny byobu to check available updates
sudo sed -i "s/updates_available//" /usr/share/byobu/status/status
# sudo sed -i "s/updates_available//" /home/pi/.byobu/status

# install Monkey web-server
cd /home/pi
git clone https://github.com/monkey/monkey.git
cd monkey
git checkout v1.6.9
./configure --malloc-libc --local
make
sudo setcap 'cap_net_bind_service=+ep' ./build/monkey  # allow using 80 port
rm build/conf/sites/default
ln -s /home/pi/catkin_ws/src/clever/deploy/monkey ./build/conf/sites/default
cd /home/pi

# install and enable Butterfly (web terminal)
sudo apt-get install libffi-dev
sudo pip3 install butterfly
sudo pip3 install butterfly[systemd]
sudo ln -s /home/pi/catkin_ws/src/clever/deploy/butterfly.service /lib/systemd/system/
sudo ln -s /home/pi/catkin_ws/src/clever/deploy/butterfly.socket /lib/systemd/system/
sudo systemctl enable butterfly.socket

echo -e "\033[0;31m\033[1m$(date) | #2 Adding mjpg-streamer at /home/pi\033[0m\033[0m"
# https://github.com/jacksonliam/mjpg-streamer

git clone https://github.com/jacksonliam/mjpg-streamer.git /home/pi/mjpg-streamer \
  && cd /home/pi/mjpg-streamer/mjpg-streamer-experimental \
  && make \
  && make install \
  && chown -Rf pi:pi /home/pi/mjpg-streamer

echo -e "\033[0;31m\033[1m$(date) | Miscellaneous\033[0m\033[0m"

# vim settings
echo "set mouse-=a
syntax on
autocmd BufNewFile,BufRead *.launch set syntax=xml
" > /home/pi/.vimrc

echo -e "\033[0;31m\033[1m$(date) | End of network installation\033[0m\033[0m"
