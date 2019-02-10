#!/usr/bin/env bash

#
# Validate built image using tests
#
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Oleg Kalachev <okalachev@gmail.com>
#

set -ex

echo "Run image tests"

export ROS_DISTRO='kinetic'
export ROS_IP='127.0.0.1'
source /opt/ros/kinetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash

cd /home/pi/catkin_ws/src/clever/builder/test/
./tests.sh
./tests.py
