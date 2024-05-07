#! /usr/bin/env bash

#
# Script for installing software to the image.
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

. /etc/os-release # set $VERSION_CODENAME to Debian release code name

# https://gist.github.com/letmaik/caa0f6cc4375cbfcc1ff26bd4530c2a3
# https://github.com/travis-ci/travis-build/blob/master/lib/travis/build/templates/header.sh
my_travis_retry() {
  local result=0
  local count=1
  while [ $count -le 3 ]; do
    [ $result -ne 0 ] && {
      echo -e "\n${ANSI_RED}The command \"$*\" failed. Retrying, $count of 3.${ANSI_RESET}\n" >&2
    }
    # ! { } ignores set -e, see https://stackoverflow.com/a/4073372
    ! { "$@"; result=$?; }
    [ $result -eq 0 ] && break
    count=$(($count + 1))
    sleep 1
  done

  [ $count -gt 3 ] && {
    echo -e "\n${ANSI_RED}The command \"$*\" failed 3 times.${ANSI_RESET}\n" >&2
  }

  return $result
}

echo "--- Increase apt retries"
echo "APT::Acquire::Retries \"3\";" > /etc/apt/apt.conf.d/80-retries

echo "--- Install apt keys & repos"

# TODO: This STDOUT consist 'OK'
apt-get update \
&& apt-get install --no-install-recommends -y dirmngr > /dev/null \
&& apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-latest.list

wget -O - 'http://packages.coex.tech/key.asc' | apt-key add -
echo "deb http://packages.coex.tech $VERSION_CODENAME main" >> /etc/apt/sources.list

echo "--- Update apt cache"

# TODO: FIX ERROR: /usr/bin/apt-key: 596: /usr/bin/apt-key: cannot create /dev/null: Permission denied
apt-get update
# && apt upgrade -y

# Let's retry fetching those packages several times, just in case
echo "--- Install software"
my_travis_retry apt-get install --no-install-recommends -y cmake-data cmake
my_travis_retry apt-get install --no-install-recommends -y \
unzip \
zip \
ipython3 \
screen \
byobu  \
nmap \
lsof \
git \
dnsmasq  \
tmux \
tree \
vim \
tcpdump \
libpoco-dev \
libzbar0 \
python3-rosinstall-generator \
python3-wstool \
python3-rosinstall \
build-essential \
libffi-dev \
monkey \
pigpio python3-pigpio \
i2c-tools \
espeak espeak-data python3-espeak \
ntpdate \
mjpg-streamer \
xxd \
python3-dev \
python3-systemd \
python3-opencv \
python3-pip
#libjpeg8 \

# Deny byobu to check available updates
sed -i "s/updates_available//" /usr/share/byobu/status/status
# sed -i "s/updates_available//" /home/pi/.byobu/status

echo "--- Make sure pip is installed"
pip --version
pip3 --version

echo "--- Enable installing packages with pip"
mv /usr/lib/python3.11/EXTERNALLY-MANAGED /usr/lib/python3.11/EXTERNALLY-MANAGED.old

echo "--- Install and enable Butterfly (web terminal)"
export CRYPTOGRAPHY_DONT_BUILD_RUST=1
my_travis_retry pip3 install cryptography==3.4.6 # https://stackoverflow.com/a/68472128/6850197
#my_travis_retry pip3 install pyOpenSSL==20.0.1
#my_travis_retry pip3 install tornado==5.1.1
my_travis_retry pip3 install butterfly
my_travis_retry pip3 install butterfly[systemd]
systemctl enable butterfly.socket

echo "--- Install ws281x library"
my_travis_retry pip3 install --prefer-binary rpi_ws281x

echo "--- Setup Monkey"
mv /etc/monkey/sites/default /etc/monkey/sites/default.orig
mv /root/monkey /etc/monkey/sites/default
sed -i 's/SymLink Off/SymLink On/' /etc/monkey/monkey.conf
systemctl enable monkey.service

echo "--- Install Node.js"
cd /home/pi
wget --no-verbose https://nodejs.org/dist/v10.15.0/node-v10.15.0-linux-armv6l.tar.gz
tar -xzf node-v10.15.0-linux-armv6l.tar.gz
cp -R node-v10.15.0-linux-armv6l/* /usr/local/
rm -rf node-v10.15.0-linux-armv6l/
rm node-v10.15.0-linux-armv6l.tar.gz

echo "--- Installing debugpy"
my_travis_retry pip3 install debugpy

echo "--- Installing pyzbar"
my_travis_retry pip3 install pyzbar

echo "--- Add .vimrc"
cat << EOF > /home/pi/.vimrc
set mouse-=a
syntax on
autocmd BufNewFile,BufRead *.launch set syntax=xml
EOF

echo "--- Change default keyboard layout to US"
sed -i 's/XKBLAYOUT="gb"/XKBLAYOUT="us"/g' /etc/default/keyboard

echo "--- Attempting to kill dirmngr"
gpgconf --kill dirmngr
# dirmngr is only used by apt-key, so we can safely kill it.
# We ignore pkill's exit value as well.
pkill -9 -f dirmngr || true
