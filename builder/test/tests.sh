#!/usr/bin/env bash

set -ex

# TODO: validate versions

# validate required software is installed

python --version
python2 --version
python3 --version
ipython --version
ipython3 --version

node -v
npm -v

byobu --version
nmap --version
lsof -v
git --version
vim --version
pip --version
pip2 --version
pip3 --version
tcpdump --version
monkey --version
pigpiod -v
i2cdetect -V
butterfly -h
espeak --version
mjpg_streamer --version

# ros stuff

roscore -h
rosversion clever
rosversion clever # old name
rosversion aruco_pose
rosversion vl53l1x
rosversion mavros
rosversion mavros_extras
rosversion dynamic_reconfigure
rosversion tf2_web_republisher
rosversion compressed_image_transport
rosversion rosbridge_suite
rosversion rosserial
rosversion usb_cam
rosversion cv_camera
rosversion web_video_server
rosversion rosshow
