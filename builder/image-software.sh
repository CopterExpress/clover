#! /usr/bin/env bash

#
# Script for install software to the image.
#
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Artem Smirnov <urpylka@gmail.com>
#

set -e # Exit immidiately on non-zero result

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

  TEXT="$(date '+[%Y-%m-%d %H:%M:%S]') $1"
  TEXT="\e[1m${TEXT}\e[0m" # BOLD

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

# https://gist.github.com/letmaik/caa0f6cc4375cbfcc1ff26bd4530c2a3
# https://github.com/travis-ci/travis-build/blob/master/lib/travis/build/templates/header.sh
my_travis_retry() {
  local result=0
  local count=1
  while [ $count -le 3 ]; do
    [ $result -ne 0 ] && {
      echo -e "\n${ANSI_RED}The command \"$@\" failed. Retrying, $count of 3.${ANSI_RESET}\n" >&2
    }
    # ! { } ignores set -e, see https://stackoverflow.com/a/4073372
    ! { "$@"; result=$?; }
    [ $result -eq 0 ] && break
    count=$(($count + 1))
    sleep 1
  done

  [ $count -gt 3 ] && {
    echo -e "\n${ANSI_RED}The command \"$@\" failed 3 times.${ANSI_RESET}\n" >&2
  }

  return $result
}

echo_stamp "Install apt keys & repos"

# TODO: This STDOUT consist 'OK'
curl http://repo.coex.space/aptly_repo_signing.key 2> /dev/null | apt-key add -
apt-get update \
&& apt-get install --no-install-recommends -y -qq dirmngr=2.1.18-8~deb9u2 > /dev/null \
&& apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

echo "deb http://packages.ros.org/ros/ubuntu stretch main" > /etc/apt/sources.list.d/ros-latest.list
echo "deb http://repo.coex.space/rpi-ros-kinetic stretch main" > /etc/apt/sources.list.d/rpi-ros-kinetic.list
echo "deb http://repo.coex.space/clever stretch main" > /etc/apt/sources.list.d/clever.list

echo_stamp "Update apt cache"

# TODO: FIX ERROR: /usr/bin/apt-key: 596: /usr/bin/apt-key: cannot create /dev/null: Permission denied
apt-get update -qq
# && apt upgrade -y

echo_stamp "Software installing"
apt-get install --no-install-recommends -y \
unzip=6.0-21 \
zip=3.0-11 \
ipython=5.1.0-3 \
ipython3=5.1.0-3 \
screen=4.5.0-6 \
byobu=5.112-1  \
nmap=7.40-1 \
lsof=4.89+dfsg-0.1 \
git \
dnsmasq=2.76-5+rpt1+deb9u1  \
tmux=2.3-4 \
vim=2:8.0.0197-4+deb9u1 \
cmake=3.7.2-1 \
python-pip=9.0.1-2+rpt2 \
python3-pip=9.0.1-2+rpt2 \
libjpeg8-dev=8d1-2 \
tcpdump \
ltrace \
libpoco-dev=1.7.6+dfsg1-5+deb9u1 \
python-rosdep=0.12.2-1 \
python-rosinstall-generator=0.1.14-1 \
python-wstool=0.1.17-1 \
python-rosinstall=0.7.8-1 \
build-essential=12.3 \
libffi-dev \
monkey=1.6.9-1 \
&& echo_stamp "Everything was installed!" "SUCCESS" \
|| (echo_stamp "Some packages wasn't installed!" "ERROR"; exit 1)

# Deny byobu to check available updates
sudo sed -i "s/updates_available//" /usr/share/byobu/status/status
# sudo sed -i "s/updates_available//" /home/pi/.byobu/status

echo_stamp "Upgrade pip"
my_travis_retry pip install --upgrade pip
my_travis_retry pip3 install --upgrade pip3

echo_stamp "Install and enable Butterfly (web terminal)"
my_travis_retry pip3 install butterfly
my_travis_retry pip3 install butterfly[systemd]
ln -s /root/butterfly.service /lib/systemd/system/
ln -s /root/butterfly.socket /lib/systemd/system/
systemctl enable butterfly.socket

echo_stamp "Setup Monkey"
mv /etc/monkey/sites/default /etc/monkey/sites/default.orig
ln -s /root/monkey-clever /etc/monkey/sites/default

echo_stamp "Add .vimrc"
cat << EOF > /home/pi/.vimrc
set mouse-=a
syntax on
autocmd BufNewFile,BufRead *.launch set syntax=xml
EOF

echo_stamp "End of software installation"
