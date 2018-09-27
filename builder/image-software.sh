#! /usr/bin/env bash

#
# Script for image configure
# @urpylka Artem Smirnov
#

# Exit immidiately on non-zero result
set -e

##################################################################################################################################
# Image software installation
##################################################################################################################################

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
git=1:2.11.0-3+deb9u3 \
dnsmasq=2.76-5+rpt1+deb9u1  \
tmux=2.3-4 \
vim=2:8.0.0197-4+deb9u1 \
cmake=3.7.2-1 \
python-pip=9.0.1-2+rpt2 \
python3-pip=9.0.1-2+rpt2 \
libjpeg8-dev=8d1-2 \
tcpdump \
libpoco-dev=1.7.6+dfsg1-5+deb9u1 \
python-rosdep=0.12.2-1 \
python-rosinstall-generator=0.1.14-1 \
python-wstool=0.1.17-1 \
python-rosinstall=0.7.8-1 \
build-essential=12.3 \
mjpg-streamer=2.0 \
python-dev \
libxml2-dev=2.9.4+dfsg1-2.2+deb9u2 \
libxslt1-dev=1.1.29-2.1 \
> /dev/null \
&& echo_stamp "Everything was installed!" "SUCCESS" \
|| (echo_stamp "Some packages wasn't installed!" "ERROR"; exit 1)

echo_stamp "Add .vimrc"
cat << EOF > /home/pi/.vimrc
set mouse-=a
syntax on
EOF

echo_stamp "End of software installation"
