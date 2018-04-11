#!/bin/bash

set -e

##################################################################################################################################
# ROS for user pi
##################################################################################################################################

# ros http://wiki.ros.org/action/fullsearch/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi
# maintainer @urpylka

echo "\033[0;31m\033[1m$(date) | #0 Installing ROS\033[0m\033[0m"

echo "\033[0;31m\033[1m$(date) | #1 Installing dirmngr & add key to apt-key\033[0m\033[0m"

# Install a tool that apt-key uses to add ROS repository key
# http://wpblogger.su/tags/apt/
apt-get install dirmngr
# setup keys
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
echo "deb http://packages.ros.org/ros/ubuntu stretch main" > /etc/apt/sources.list.d/ros-latest.list

echo "\033[0;31m\033[1m$(date) | #2 apt update && apt upgrade\033[0m\033[0m"

# install bootstrap tools
apt-get update
# && apt upgrade -y

echo "\033[0;31m\033[1m$(date) | #3 Installing wget, unzip, python-rosdep, python-rosinstall-generator, python-wstool, python-rosinstall, build-essential, cmake\033[0m\033[0m"

apt-get install --no-install-recommends -y \
  wget \
  unzip \
  python-rosdep \
  python-rosinstall-generator \
  python-wstool \
  python-rosinstall \
  build-essential \
  cmake \
  libjpeg8-dev

echo "\033[0;31m\033[1m$(date) | #4 rosdep init && rosdep update\033[0m\033[0m"

# bootstrap rosdep
rosdep init && rosdep update

echo "\033[0;31m\033[1m$(date) | #5 Preparing ros_comm packages to kinetic-ros_comm-wet.rosinstall\033[0m\033[0m"

# create catkin workspace
mkdir -p /home/pi/ros_catkin_ws && cd /home/pi/ros_catkin_ws \
    && rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall \
    && wstool init src kinetic-ros_comm-wet.rosinstall

echo "\033[0;31m\033[1m$(date) | #7 Preparing other ROS-packages to kinetic-custom_ros.rosinstall\033[0m\033[0m"

cd /home/pi/ros_catkin_ws \
  && rosinstall_generator \
  actionlib actionlib_msgs angles async_web_server_cpp bond bond_core bondcpp bondpy camera_calibration_parsers camera_info_manager catkin class_loader cmake_modules cpp_common cv_bridge cv_camera diagnostic_msgs diagnostic_updater dynamic_reconfigure eigen_conversions gencpp geneus genlisp genmsg gennodejs genpy geographic_msgs geometry_msgs geometry2 image_transport libmavconn mavlink mavros_msgs message_filters message_generation message_runtime mk nav_msgs nodelet orocos_kdl pluginlib python_orocos_kdl ros ros_comm rosapi rosauth rosbag rosbag_migration_rule rosbag_storage rosbash rosboost_cfg rosbridge_library rosbridge_server rosbridge_suite rosbuild rosclean rosconsole rosconsole_bridge roscpp roscpp_serialization roscpp_traits roscreate rosgraph rosgraph_msgs roslang roslaunch roslib roslint roslisp roslz4 rosmake rosmaster rosmsg rosnode rosout rospack rosparam rospy rospy_tutorials rosserial rosserial_client rosserial_msgs rosserial_python rosservice rostest rostime rostopic rosunit roswtf sensor_msgs smclib std_msgs std_srvs stereo_msgs tf tf2 tf2_bullet tf2_eigen tf2_geometry_msgs tf2_kdl tf2_msgs tf2_py tf2_ros tf2_sensor_msgs tf2_tools topic_tools trajectory_msgs urdf urdf_parser_plugin usb_cam uuid_msgs visualization_msgs web_video_server xmlrpcpp mavros opencv3 mavros_extras \
  --rosdistro kinetic --deps --wet-only --tar > kinetic-custom_ros.rosinstall \
  && wstool merge -t src kinetic-custom_ros.rosinstall \
  && wstool update -t src

echo "\033[0;31m\033[1m$(date) | #8 Installing dependencies apps with rosdep\033[0m\033[0m"

# Resolving Dependencies with rosdep
cd /home/pi/ros_catkin_ws \
  && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:stretch

echo "\033[0;31m\033[1m$(date) | #9 Refactoring usb_cam in SRC\033[0m\033[0m"

sed -i '/#define __STDC_CONSTANT_MACROS/a\#define PIX_FMT_RGB24 AV_PIX_FMT_RGB24\n#define PIX_FMT_YUV422P AV_PIX_FMT_YUV422P' /home/pi/ros_catkin_ws/src/usb_cam/src/usb_cam.cpp

echo "\033[0;31m\033[1m$(date) | #10 Installing GeographicLib datasets\033[0m\033[0m"

/home/pi/ros_catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

#echo "\033[0;31m\033[1m$(date) | #11 Building light packages on 2 threads\033[0m\033[0m"

# Build the catkin Workspace
#cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -j2 --install-space /opt/ros/kinetic --pkg actionlib actionlib_msgs angles async_web_server_cpp bond bond_core bondcpp bondpy camera_calibration_parsers camera_info_manager catkin class_loader cmake_modules cpp_common diagnostic_msgs diagnostic_updater dynamic_reconfigure eigen_conversions gencpp geneus genlisp genmsg gennodejs genpy geographic_msgs geometry_msgs geometry2 image_transport libmavconn mavlink mavros_msgs message_filters message_generation message_runtime mk nav_msgs nodelet orocos_kdl pluginlib python_orocos_kdl ros ros_comm rosapi rosauth rosbag rosbag_migration_rule rosbag_storage rosbash rosboost_cfg rosbridge_library rosbridge_server rosbridge_suite rosbuild rosclean rosconsole rosconsole_bridge roscpp roscpp_serialization roscpp_traits roscreate rosgraph rosgraph_msgs roslang roslaunch roslib roslint roslisp roslz4 rosmake rosmaster rosmsg rosnode rosout rospack rosparam rospy rospy_tutorials rosserial rosserial_client rosserial_msgs rosserial_python rosservice rostest rostime rostopic rosunit roswtf sensor_msgs smclib std_msgs std_srvs stereo_msgs tf tf2 tf2_bullet tf2_eigen tf2_geometry_msgs tf2_kdl tf2_msgs tf2_py tf2_ros tf2_sensor_msgs tf2_tools topic_tools trajectory_msgs urdf urdf_parser_plugin usb_cam uuid_msgs visualization_msgs xmlrpcpp

#echo "\033[0;31m\033[1m$(date) | #12 Building heavy packages\033[0m\033[0m"

# This command uses less threads to avoid Raspberry Pi freeze
# Build the catkin Workspace
#cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -j1 --install-space /opt/ros/kinetic --pkg mavros opencv3 cv_bridge cv_camera mavros_extras web_video_server

echo "\033[0;31m\033[1m$(date) | #11 Building packages on 1 thread\033[0m\033[0m"

# Install builded packages
# WARNING: A major bug was found when using --pkg option (catkin_make_isolated doesn't install environment files)
# TODO: Can we increase threads number with HDD swap?
cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -j1 -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic

echo "\033[0;31m\033[1m$(date) | #13 Creating catkin_ws\033[0m\033[0m"

mkdir -p /home/pi/catkin_ws/src \
  && cd /home/pi/catkin_ws \
  && . /opt/ros/kinetic/setup.sh \
  && catkin init \
  && wstool init /home/pi/catkin_ws/src

echo "\033[0;31m\033[1m$(date) | #14 Installing CLEVER-BUNDLE\033[0m\033[0m"

cd /home/pi/catkin_ws/src \
  && git clone https://github.com/CopterExpress/clever.git clever \
  && pip install wheel \
  && pip install -r /home/pi/catkin_ws/src/clever/clever/requirements.txt \
  && cd /home/pi/catkin_ws \
  && . /opt/ros/kinetic/setup.sh \
  && catkin_make -j1 \
  && systemctl enable /home/pi/catkin_ws/src/clever/deploy/roscore.service \
  && systemctl enable /home/pi/catkin_ws/src/clever/deploy/clever.service

echo "\033[0;31m\033[1m$(date) | #15 Adding mjpg-streamer at /home/pi\033[0m\033[0m"

# https://github.com/jacksonliam/mjpg-streamer

cd /home/pi \
  && git clone https://github.com/jacksonliam/mjpg-streamer.git \
  && cd /home/pi/mjpg-streamer/mjpg-streamer-experimental \
  && make \
  && make install

echo "\033[0;31m\033[1m$(date) | #16 Adding ENV vars\033[0m\033[0m"

# setup environment
echo "LANG=C.UTF-8" >> /home/pi/.bashrc
echo "LC_ALL=C.UTF-8" >> /home/pi/.bashrc
echo "ROS_DISTRO=kinetic" >> /home/pi/.bashrc
echo "export ROS_IP=192.168.11.1" >> /home/pi/.bashrc

echo "source /opt/ros/kinetic/setup.bash" >> /home/pi/.bashrc \
  && echo "source /home/pi/catkin_ws/devel/setup.bash" >> /home/pi/.bashrc

chown -Rf pi:pi /home/pi

#echo "\033[0;31m\033[1m$(date) | #16 Removing local apt mirror\033[0m\033[0m"
# Restore original sources.list
#mv /var/sources.list.bak /etc/apt/sources.list
# Clean apt cache
apt-get clean
# Remove local mirror repository key
#apt-key del COEX-MIRROR

echo "\033[0;31m\033[1m$(date) | #17 END of ROS INSTALLATION\033[0m\033[0m"
