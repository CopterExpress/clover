#!/bin/bash

##################################################################################################################################
# Image software installation
##################################################################################################################################

set -e

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

  TEXT="$(date) | $1"
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

echo_stamp "#1 Software installing"

apt-get install --no-install-recommends -y -qq \
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
  > /dev/null \
  && echo_stamp "Everything was installed!" "SUCCESS" \
  || (echo_stamp "Some packages wasn't installed!" "ERROR"; exit 1)

  apt search ros-kinetic-opencv3

  echo "dasfasfdsagfasdgagasgas" >/dev/null 2>&1 || true

echo_stamp "Instalation opencv"

apt-get install --no-install-recommends -y -qq \
  ros-kinetic-opencv3=3.3.1-0stretch \
  > /dev/null \
  && echo_stamp "OpenCV3 was installed!" "SUCCESS" \
  || (echo_stamp "OpenCV3 wasn't installed!" "ERROR"; exit 1)

echo_stamp "#2 Adding mjpg-streamer at /home/pi"
# https://github.com/jacksonliam/mjpg-streamer

git clone https://github.com/jacksonliam/mjpg-streamer.git /home/pi/mjpg-streamer \
&& cd /home/pi/mjpg-streamer/mjpg-streamer-experimental \
&& make > /dev/null \
&& make install \
&& chown -Rf pi:pi /home/pi/mjpg-streamer

echo_stamp "Add .vimrc"

cat << EOF > /home/pi/.vimrc
set mouse-=a
syntax on
EOF

echo_stamp "#3 End of network installation"
