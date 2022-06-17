#! /usr/bin/env bash

#
# Script for building ROS packages from scratch
#
# Copyright (C) 2022 Copter Express Technologies
#
# Author: Oleg Kalachev <okalachev@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -ex # exit on error, echo commands

# http://wiki.ros.org/Installation/Source

ROS_DISTRO=noetic
. /etc/os-release # set $VERSION_CODENAME to Debian release code name

echo "=== Building ROS from scratch"

apt-get update
apt-get install -y python3-distutils python3-rosdep python3-rosinstall-generator build-essential # python3-vcstool

# install vcstool using pip
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3 get-pip.py && rm get-pip.py
pip3 install -U vcstool

# sudo rosdep init
# rosdep update

# rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init
# rosdep --os=debian:$VERSION_CODENAME update

echo "Create Catkin workspace to build ROS package"
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

echo "Download ROS sources"
rosinstall_generator ros_base --rosdistro $ROS_DISTRO --deps --tar > noetic.rosinstall
mkdir ./src
vcs import --input noetic.rosinstall ./src

echo "Resolve dependencies"
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro $ROS_DISTRO -y

echo "Build ROS"
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

source ~/ros_catkin_ws/install_isolated/setup.bash
