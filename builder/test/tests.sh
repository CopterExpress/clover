#!/usr/bin/env bash

set -ex

# TODO: validate versions

# validate required software is installed

python2 --version
python3 --version
ipython3 --version

node -v
npm -v

byobu --version
nmap --version
lsof -v
git --version
vim --version
pip --version
pip3 --version
tcpdump --version
monkey --version
# espeak --version
systemctl --version

if [ -z $VM ]; then
	# rpi only software
	python --version
	ipython --version
	pip2 --version
	# `python` is python2 for now
	[[ $(python -c 'import sys;print(sys.version_info.major)') == "2" ]]

	# ptvsd does not have a stand-alone binary
	python -m ptvsd --version
	python3 -m ptvsd --version

	pigpiod -v
	i2cdetect -V
	butterfly -h
	mjpg_streamer --version
fi

# ros stuff

roscore -h
rosversion clover
rosversion aruco_pose
rosversion mavros
rosversion mavros_extras
rosversion ws281x
rosversion led_msgs
rosversion dynamic_reconfigure
rosversion tf2_web_republisher
rosversion rosbridge_server
rosversion rosserial
rosversion usb_cam
rosversion cv_camera
rosversion web_video_server
rosversion nodelet
rosversion image_view

[[ $(rosversion ws281x) == "0.0.13" ]]

if [ -z $VM ]; then
	rosversion compressed_image_transport
	rosversion rosshow
	rosversion vl53l1x
	[[ $(rosversion cv_camera) == "0.5.1" ]] # patched version with init fix
fi

[ $VM ] && H="/home/clover" || H="/home/pi"

# validate examples are present
[[ $(ls $H/examples/*) ]]
