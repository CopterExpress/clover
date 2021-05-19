#!/usr/bin/env bash

set -ex

# TODO: validate versions

# validate required software is installed

python --version
python2 --version
python3 --version
ipython --version
ipython3 --version

# ptvsd does not have a stand-alone binary
python -m ptvsd --version
python3 -m ptvsd --version

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
# espeak --version
mjpg_streamer --version

# ros stuff

roscore -h
rosversion clover
rosversion aruco_pose
rosversion vl53l1x
rosversion mavros
rosversion mavros_extras
rosversion ws281x
rosversion led_msgs
rosversion dynamic_reconfigure
rosversion tf2_web_republisher
rosversion compressed_image_transport
rosversion rosbridge_suite
rosversion rosserial
rosversion usb_cam
rosversion cv_camera
rosversion web_video_server
rosversion rosshow
rosversion nodelet

# validate examples are present
[[ $(ls /home/pi/examples/*) ]]
