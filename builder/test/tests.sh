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
rosversion usb_cam
rosversion cv_camera
rosversion web_video_server
rosversion nodelet
rosversion image_view
rosversion stereo_msgs
rosversion vision_msgs
rosversion angles

[[ $(rosversion ws281x) == "0.0.13" ]]

if [ -z $VM ]; then
	rosversion compressed_image_transport
	rosversion rosshow
	rosversion vl53l1x
	rosversion rosserial
	[[ $(rosversion cv_camera) == "0.5.1" ]] # patched version with init fix
fi

# determine user home directory
[ $VM ] && H="/home/clover" || H="/home/pi"

# test basic ros tool work
source $H/catkin_ws/devel/setup.bash
roscd
rosrun
rosmsg
rossrv
rosnode || [ $? -eq 64 ] # usage output code is 64
rostopic || [ $? -eq 64 ]
rosservice || [ $? -eq 64 ]
rosparam
roslaunch -h

# validate examples are present
[[ $(ls $H/examples/*) ]]

# validate web tools present
[ -d $H/.ros/www ]
[ "$(readlink $H/.ros/www/clover)" = "$H/catkin_ws/src/clover/clover/www" ]
[ "$(readlink $H/.ros/www/clover_blocks)" = "$H/catkin_ws/src/clover/clover_blocks/www" ]
