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

export ROS_DISTRO=noetic
. /etc/os-release # set $VERSION_CODENAME to Debian release code name
export ROS_OS_OVERRIDE=debian:$VERSION_CODENAME
export ROS_PYTHON_VERSION=3
export CLOVER_DEPS="tf tf2 tf2_ros tf2_geometry_msgs geometry_msgs sensor_msgs visualization_msgs libgeographiclib-dev mavros mavros_extras cv_camera cv_bridge rosbridge_server web_video_server tf2_web_republisher libxml2 libxslt python3-lxml dynamic_reconfigure image_transport image_proc image_geometry python-pymavlink ros_pytest"
export CLOVER_DEPS="$CLOVER_DEPS rostest python3-docopt image_publisher"

echo "=== Building ROS from scratch"

#echo "--- Adding sources"
# echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
# curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

#cp /etc/apt/trusted.gpg /etc/apt/trusted.gpg.d # https://askubuntu.com/a/1408456
apt-get update
apt-get install -y python3-distutils build-essential cmake git python3-pip python3-rosinstall-generator python3-vcstools python3-empy libpoco-dev

# install vcstool using pip
# curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3 get-pip.py && rm get-pip.py
pip3 install -U --break-system-packages vcstool rosdep rosinstall-generator catkin-pkg

# sudo rosdep init
# rm /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init
rosdep update --os=debian:bullseye

# rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init
# rosdep --os=debian:$VERSION_CODENAME update

echo "--- Create Catkin workspace to build ROS package"
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

echo "--- Download ROS sources"
rosinstall_generator ros_base $CLOVER_DEPS --rosdistro $ROS_DISTRO --deps --tar > noetic.rosinstall
mkdir ./src
vcs import --input noetic.rosinstall ./src

# https://answers.ros.org/question/343367/catkin-package-dependencies-issue-when-installing-ros-melodic-on-raspberry-pi-4/
#sudo apt remove python-rospkg
#sudo apt remove python-catkin-pkg
##sudo apt --fix-broken install
#sudo apt-get autoremove

#echo "--- Install catkin_pkg"
#cd
#git clone https://github.com/ros-infrastructure/catkin_pkg.git
#cd catkin_pkg
#python3 setup.py install
#cd ~/ros_catkin_ws

echo "--- Resolve dependencies"
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro $ROS_DISTRO -y --os=debian:bullseye --skip-keys="python3-catkin-pkg-modules libboost-thread python3-rosdep-modules" || true

echo "--- Install missing dependencies"
apt-get install -y liborocos-kdl1.5 geographiclib-tools libgeographiclib-dev

echo "-- Install geographiclib datasets"
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

echo "--- Apply patches"
wget https://github.com/ros/rosconsole/pull/58.patch
patch -p1 -d src/rosconsole < 58.patch

wget https://github.com/ros/ros_comm/pull/2353.patch
patch -p2 -d src/ros_comm < 2353.patch

wget https://github.com/AJahueyM/web_video_server/commit/5b722eb0822bcc3fe45fefe7b393b87bfe004417.patch
patch -p1 -d src/web_video_server < 5b722eb0822bcc3fe45fefe7b393b87bfe004417.patch

echo "--- Build ROS"
# https://github.com/ros/catkin/issues/863#issuecomment-290392074
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
#	-DSETUPTOOLS_DEB_LAYOUT=OFF \
#	--install-space=/opt/ros/$ROS_DISTRO

# source ~/ros_catkin_ws/install_isolated/setup.bash
#source /opt/ros/$ROS_DISTRO/setup.bash
#
#echo "--- List built ROS packages"
#set +x
#rospack list-names | while read line; do echo $line `rosversion $line`; done
#set -x
#
#echo "--- Build Debian packages"
#apt-get install -y python3-bloom debhelper dpkg-dev libtinyxml-dev
#
## add rosdep file to help bloom-generate resolve missing bookworm dependencies
#echo "yaml file:///etc/ros/rosdep/noetic-bookworm.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list
#rosdep update
#
#pip3 install setuptools==45.2.0 # https://github.com/ros/catkin/issues/863#issuecomment-1000446018
#
#for file in `find . -name "package.xml" -not -path "*/debian/*"`; do
#	cd $(dirname ${file})
#	rm -rf debian
#	bloom-generate rosdebian --os-name debian --os-version $VERSION_CODENAME --ros-distro $ROS_DISTRO --debug
#	debian/rules binary # fakeroot is not needed as we are root
#	cd -
#done

ls
