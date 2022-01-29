#!/usr/bin/env bash

#
# Validate built image using tests
#
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Oleg Kalachev <okalachev@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -ex

echo "Run image tests"

export ROS_DISTRO='noetic'
export ROS_IP='127.0.0.1'
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
systemctl start roscore

cd /home/pi/catkin_ws/src/clover/builder/test/
./tests.sh
./tests.py
./tests_py3.py
[[ $(./test_qr.py) == "Found QRCODE with data Проверка Unicode with center at x=66.0, y=66.0" ]] 
[[ $(./tests_clever.py) == "Warning: clever package is renamed to clover" ]]  # test backwards compatibility

systemctl stop roscore

# check documented packages available
apt-cache show gst-rtsp-launch
apt-cache show openvpn

echo "Move /etc/ld.so.preload back to its original position"
mv /etc/ld.so.preload.disabled-for-build /etc/ld.so.preload
