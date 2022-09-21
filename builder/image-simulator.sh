#! /usr/bin/env bash

#
# Script for build the image. Used builder script of the target repo
# For build: docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/builder/repo smirart/builder
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

echo "--- Downloading PX4"
cd /home/pi/catkin_ws/
git clone --recursive --depth 1 --branch v1.13.0 https://github.com/PX4/PX4-Autopilot.git /home/pi/PX4-Autopilot
ln -s /home/pi/PX4-Autopilot /home/pi/catkin_ws/src/
ln -s /home/pi/PX4-Autopilot/Tools/sitl_gazebo /home/pi/catkin_ws/src/

echo "--- Installing PX4 dependencies"
echo "progress=dot:giga" > /home/pi/.wgetrc # make wget don't spam to log
apt-get install --no-install-recommends -y gazebo11
/home/pi/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
rm /home/pi/.wgetrc
pip3 install --user toml

echo "--- Patching mavlink_sitl_gazebo"
# See https://github.com/PX4/PX4-SITL_gazebo/pull/872
cd /home/pi/PX4-Autopilot/Tools/sitl_gazebo
patch -p1 < /tmp/patches/sitl_gazebo.patch

echo "--- Build mavlink"
cd /home/pi/catkin_ws
catkin_make mavlink_c_generate -DCATKIN_WHITELIST_PACKAGES="px4"  # at first build PX4's mavlink to enforce mavlink_sitl_gazebo using it
ln -s "." build/mavlink/mavlink  # fix https://github.com/PX4/PX4-Autopilot/pull/19964

echo "--- Building the workspace"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
