#!/bin/bash

##################################################################################################################################
# ROS for user pi
##################################################################################################################################

# ros http://wiki.ros.org/action/fullsearch/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi
# maintainer @urpylka

echo -e "\033[0;31m\033[1m$(date) | #0 Install ROS\033[0m\033[0m"




echo -e "\033[0;31m\033[1m$(date) | #1 Install dirmngr & add key to apt-key\033[0m\033[0m"

# по умолчанию dirmngr отсуствует на образе и требуется для установки ключа
# http://wpblogger.su/tags/apt/
apt install dirmngr
# setup keys
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
echo "deb http://packages.ros.org/ros/ubuntu stretch main" > /etc/apt/sources.list.d/ros-latest.list




echo -e "\033[0;31m\033[1m$(date) | #2 apt update && apt upgrade\033[0m\033[0m"

# install bootstrap tools
apt update
# && apt upgrade -y




echo -e "\033[0;31m\033[1m$(date) | #3 Install wget, unzip, python-rosdep, python-rosinstall-generator, python-wstool, python-rosinstall, build-essential, cmake\033[0m\033[0m"

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



echo -e "\033[0;31m\033[1m$(date) | #4 rosdep init && rosdep update\033[0m\033[0m"

# bootstrap rosdep
rosdep init && rosdep update




echo -e "\033[0;31m\033[1m$(date) | #5 Prepare ros_comm packages to kinetic-ros_comm-wet.rosinstall\033[0m\033[0m"

# create catkin workspace
mkdir -p /home/pi/ros_catkin_ws && cd /home/pi/ros_catkin_ws \
    && rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall \
    && wstool init src kinetic-ros_comm-wet.rosinstall



echo -e "\033[0;31m\033[1m$(date) | #6 Install assimp-3.1.1 to /home/pi/ros_catkin_ws/external_src\033[0m\033[0m"

# Unavailable Dependencies
mkdir -p /home/pi/ros_catkin_ws/external_src \
    && cd /home/pi/ros_catkin_ws/external_src \
    && wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip \
    && unzip assimp-3.1.1_no_test_models.zip \
    && cd assimp-3.1.1 \
    && cmake . \
    && make \
    && make install




echo -e "\033[0;31m\033[1m$(date) | #7 Prepare other ROS-packages to kinetic-custom_ros.rosinstall\033[0m\033[0m"

cd /home/pi/ros_catkin_ws \
  && rosinstall_generator \
  actionlib actionlib_msgs angles async_web_server_cpp bond bond_core bondcpp bondpy camera_calibration_parsers camera_info_manager catkin class_loader cmake_modules cpp_common cv_bridge cv_camera diagnostic_msgs diagnostic_updater dynamic_reconfigure eigen_conversions gencpp geneus genlisp genmsg gennodejs genpy geographic_msgs geometry_msgs geometry2 image_transport libmavconn mavlink mavros_msgs message_filters message_generation message_runtime mk nav_msgs nodelet orocos_kdl pluginlib python_orocos_kdl ros ros_comm rosapi rosauth rosbag rosbag_migration_rule rosbag_storage rosbash rosboost_cfg rosbridge_library rosbridge_server rosbridge_suite rosbuild rosclean rosconsole rosconsole_bridge roscpp roscpp_serialization roscpp_traits roscreate rosgraph rosgraph_msgs roslang roslaunch roslib roslint roslisp roslz4 rosmake rosmaster rosmsg rosnode rosout rospack rosparam rospy rospy_tutorials rosserial rosserial_client rosserial_msgs rosserial_python rosservice rostest rostime rostopic rosunit roswtf sensor_msgs smclib std_msgs std_srvs stereo_msgs tf tf2 tf2_bullet tf2_eigen tf2_geometry_msgs tf2_kdl tf2_msgs tf2_py tf2_ros tf2_sensor_msgs tf2_tools topic_tools trajectory_msgs urdf urdf_parser_plugin usb_cam uuid_msgs visualization_msgs web_video_server xmlrpcpp mavros opencv3 mavros_extras \
  --rosdistro kinetic --deps --wet-only --tar > kinetic-custom_ros.rosinstall \
  && wstool merge -t src kinetic-custom_ros.rosinstall \
  && wstool update -t src




echo -e "\033[0;31m\033[1m$(date) | #8 Install dependencies apps with rosdep\033[0m\033[0m"

# как я понял установка apt-get всяких зависимостей для ros-пакетов
# Resolving Dependencies with rosdep
cd /home/pi/ros_catkin_ws \
  && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:stretch




echo -e "\033[0;31m\033[1m$(date) | #9 Refactor usb_cam in SRC\033[0m\033[0m"

# добавление префикса с помощью двух define
# #define PIX_FMT_RGB24 AV_PIX_FMT_RGB24
# #define PIX_FMT_YUV422P AV_PIX_FMT_YUV422P

sed -i '/#define __STDC_CONSTANT_MACROS/a\#define PIX_FMT_RGB24 AV_PIX_FMT_RGB24\n#define PIX_FMT_YUV422P AV_PIX_FMT_YUV422P' /home/pi/ros_catkin_ws/src/usb_cam/src/usb_cam.cpp



echo -e "\033[0;31m\033[1m$(date) | #10 Install GeographicLib datasets\033[0m\033[0m"

/home/pi/ros_catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh




echo -e "\033[0;31m\033[1m$(date) | #11 Build light packages on 2 threads\033[0m\033[0m"

# Building the catkin Workspace
cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic -j2 --pkg actionlib actionlib_msgs angles async_web_server_cpp bond bond_core bondcpp bondpy camera_calibration_parsers camera_info_manager catkin class_loader cmake_modules cpp_common diagnostic_msgs diagnostic_updater dynamic_reconfigure eigen_conversions gencpp geneus genlisp genmsg gennodejs genpy geographic_msgs geometry_msgs geometry2 image_transport libmavconn mavlink mavros_msgs message_filters message_generation message_runtime mk nav_msgs nodelet orocos_kdl pluginlib python_orocos_kdl ros ros_comm rosapi rosauth rosbag rosbag_migration_rule rosbag_storage rosbash rosboost_cfg rosbridge_library rosbridge_server rosbridge_suite rosbuild rosclean rosconsole rosconsole_bridge roscpp roscpp_serialization roscpp_traits roscreate rosgraph rosgraph_msgs roslang roslaunch roslib roslint roslisp roslz4 rosmake rosmaster rosmsg rosnode rosout rospack rosparam rospy rospy_tutorials rosserial rosserial_client rosserial_msgs rosserial_python rosservice rostest rostime rostopic rosunit roswtf sensor_msgs smclib std_msgs std_srvs stereo_msgs tf tf2 tf2_bullet tf2_eigen tf2_geometry_msgs tf2_kdl tf2_msgs tf2_py tf2_ros tf2_sensor_msgs tf2_tools topic_tools trajectory_msgs urdf urdf_parser_plugin usb_cam uuid_msgs visualization_msgs xmlrpcpp




echo -e "\033[0;31m\033[1m$(date) | #12 Build heavy packages\033[0m\033[0m"

# Building the catkin Workspace
cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic -j1 --pkg mavros opencv3 cv_bridge cv_camera mavros_extras web_video_server



echo -e "\033[0;31m\033[1m$(date) | #13 Create catkin_ws\033[0m\033[0m"

mkdir -p /home/pi/catkin_ws/src \
  && cd /home/pi/catkin_ws \
  && source /opt/ros/kinetic/setup.bash \
  && catkin init \
  && wstool init /home/pi/catkin_ws/src




echo -e "\033[0;31m\033[1m$(date) | #14 Install CLEVER-BUNDLE\033[0m\033[0m"

cd /home/pi/catkin_ws/src \
  && git clone https://github.com/CopterExpress/clever.git clever \
  && pip install wheel \
  && pip install -r /home/pi/catkin_ws/src/clever/clever/requirements.txt \
  && cd /home/pi/catkin_ws \
  && source /opt/ros/kinetic/setup.bash \
  && catkin_make -j1 \
  && systemctl enable /home/pi/catkin_ws/src/clever/deploy/roscore.service \
  && systemctl enable /home/pi/catkin_ws/src/clever/deploy/clever.service



echo -e "\033[0;31m\033[1m$(date) | #15 Add mjpg-streamer at /home/pi\033[0m\033[0m"

# https://github.com/jacksonliam/mjpg-streamer

cd /home/pi \
  && git clone https://github.com/jacksonliam/mjpg-streamer.git \
  && cd /home/pi/mjpg-streamer/mjpg-streamer-experimental \
  && make \
  && make install



echo -e "\033[0;31m\033[1m$(date) | #16 Add ENV vars\033[0m\033[0m"

# setup environment
echo "LANG=C.UTF-8" >> /home/pi/.bashrc
echo "LC_ALL=C.UTF-8" >> /home/pi/.bashrc
echo "ROS_DISTRO=kinetic" >> /home/pi/.bashrc
echo "export ROS_IP=192.168.11.1" >> /home/pi/.bashrc

echo "source /opt/ros/kinetic/setup.bash" >> /home/pi/.bashrc \
  && echo "source /home/pi/catkin_ws/devel/setup.bash" >> /home/pi/.bashrc

chown -Rf pi:pi /home/pi



echo -e "\033[0;31m\033[1m$(date) | #17 END of ROS INSTALLATION\033[0m\033[0m"
