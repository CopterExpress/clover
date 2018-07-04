#!/bin/bash

set -e

##################################################################################################################################
# ROS for user pi
##################################################################################################################################

# ros http://wiki.ros.org/action/fullsearch/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi
# maintainer @urpylka

echo -e "\033[0;31m\033[1m$(date) | Installing ROS\033[0m\033[0m"

echo -e "\033[0;31m\033[1m$(date) | #1 Installing dirmngr & add key to apt-key\033[0m\033[0m"

# Install a tool that apt-key uses to add ROS repository key
# http://wpblogger.su/tags/apt/
apt-get install --no-install-recommends -y dirmngr=2.1.18-8~deb9u2
# setup keys
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
echo "deb http://packages.ros.org/ros/ubuntu stretch main" > /etc/apt/sources.list.d/ros-latest.list

echo -e "\033[0;31m\033[1m$(date) | #2 apt update && apt upgrade\033[0m\033[0m"

# install bootstrap tools
apt-get update
# && apt upgrade -y

echo -e "\033[0;31m\033[1m$(date) | #3 Installing wget, unzip, python-rosdep, python-rosinstall-generator, python-wstool, python-rosinstall, build-essential, cmake\033[0m\033[0m"

apt-get install --no-install-recommends -y \
  python-rosdep=0.12.2-1 \
  python-rosinstall-generator=0.1.14-1 \
  python-wstool=0.1.17-1 \
  python-rosinstall=0.7.8-1 \
  build-essential=12.3

echo -e "\033[0;31m\033[1m$(date) | #4 rosdep init && rosdep update\033[0m\033[0m"

# bootstrap rosdep
rosdep init && rosdep update

# If $2 = false, then discover packages
if [ "$2" = "false" ];
then
  echo -e "\033[0;31m\033[1m$(date) | #5 Preparing ros_comm packages to kinetic-ros_comm-wet.rosinstall\033[0m\033[0m"

  # create ros catkin workspace
  mkdir -p /home/pi/ros_catkin_ws && cd /home/pi/ros_catkin_ws \
      && rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall \
      && wstool init src kinetic-ros_comm-wet.rosinstall

  echo -e "\033[0;31m\033[1m$(date) | #6 Preparing other ROS-packages to kinetic-custom_ros.rosinstall\033[0m\033[0m"

  cd /home/pi/ros_catkin_ws \
    && rosinstall_generator \
    actionlib actionlib_msgs angles async_web_server_cpp bond bond_core bondcpp bondpy camera_calibration_parsers camera_info_manager catkin class_loader cmake_modules cpp_common cv_bridge cv_camera diagnostic_msgs diagnostic_updater dynamic_reconfigure eigen_conversions gencpp geneus genlisp genmsg gennodejs genpy geographic_msgs geometry_msgs geometry2 image_transport libmavconn mavlink mavros_msgs message_filters message_generation message_runtime mk nav_msgs nodelet orocos_kdl pluginlib python_orocos_kdl ros ros_comm rosapi rosauth rosbag rosbag_migration_rule rosbag_storage rosbash rosboost_cfg rosbridge_library rosbridge_server rosbridge_suite rosbuild rosclean rosconsole rosconsole_bridge roscpp roscpp_serialization roscpp_traits roscreate rosgraph rosgraph_msgs roslang roslaunch roslib roslint roslisp roslz4 rosmake rosmaster rosmsg rosnode rosout rospack rosparam rospy rospy_tutorials rosserial rosserial_client rosserial_msgs rosserial_python rosservice rostest rostime rostopic rosunit roswtf sensor_msgs smclib std_msgs std_srvs stereo_msgs tf tf2 tf2_bullet tf2_eigen tf2_geometry_msgs tf2_kdl tf2_msgs tf2_py tf2_ros tf2_sensor_msgs tf2_tools topic_tools trajectory_msgs urdf urdf_parser_plugin usb_cam uuid_msgs visualization_msgs web_video_server xmlrpcpp mavros opencv3 mavros_extras \
    --rosdistro kinetic --deps --wet-only --tar > kinetic-custom_ros.rosinstall \
    && wstool merge -t src kinetic-custom_ros.rosinstall \
    && wstool update -t src
else
  echo -e "\033[0;31m\033[1m$(date) | #5 Creating manual ros_catkin_ws\033[0m\033[0m"

  mkdir -p /home/pi/ros_catkin_ws && cd /home/pi/ros_catkin_ws \
    && mv /root/kinetic-ros-coex.rosinstall kinetic-ros-coex.rosinstall \
    && wstool init src kinetic-ros-coex.rosinstall
fi

echo -e "\033[0;31m\033[1m$(date) | #7 Installing dependencies apps with rosdep\033[0m\033[0m"
cd /home/pi/ros_catkin_ws
# There is a risk that umount will fail
set +e
# Successfull unmount flag (false at thismoment)
install_ok=false
# Repeat 5 times
for i in {1..5}
do
  # Resolving Dependencies with rosdep
  rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:stretch
  # If no problems detected
  if [[ $? == 0 ]]
  then
    echo -e "\033[0;31m\033[1m$(date) | Successfull rosdep install\033[0m\033[0m"
    # Set flag
    install_ok=true
    # Exit loop
    break
  fi
  # Unmount has failed
  echo -e "\033[0;31m\033[1m$(date) | Rosdep installation failed\033[0m\033[0m"
  # Wait for some time
  sleep 2
done
set -e
# Jenkins job will fail if this condition is not true
[[ "$install_ok" == true ]]
echo -e "\033[0;31m\033[1m$(date) | End of rosdep install\033[0m\033[0m"

echo -e "\033[0;31m\033[1m$(date) | #8 Refactoring usb_cam in SRC\033[0m\033[0m"

sed -i '/#define __STDC_CONSTANT_MACROS/a\#define PIX_FMT_RGB24 AV_PIX_FMT_RGB24\n#define PIX_FMT_YUV422P AV_PIX_FMT_YUV422P' /home/pi/ros_catkin_ws/src/usb_cam/src/usb_cam.cpp

echo -e "\033[0;31m\033[1m$(date) | #9 Installing GeographicLib datasets\033[0m\033[0m"

/home/pi/ros_catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

#echo -e "\033[0;31m\033[1m$(date) | #11 Building light packages on 2 threads\033[0m\033[0m"

# Build the catkin Workspace
#cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -j2 --install-space /opt/ros/kinetic --pkg actionlib actionlib_msgs angles async_web_server_cpp bond bond_core bondcpp bondpy camera_calibration_parsers camera_info_manager catkin class_loader cmake_modules cpp_common diagnostic_msgs diagnostic_updater dynamic_reconfigure eigen_conversions gencpp geneus genlisp genmsg gennodejs genpy geographic_msgs geometry_msgs geometry2 image_transport libmavconn mavlink mavros_msgs message_filters message_generation message_runtime mk nav_msgs nodelet orocos_kdl pluginlib python_orocos_kdl ros ros_comm rosapi rosauth rosbag rosbag_migration_rule rosbag_storage rosbash rosboost_cfg rosbridge_library rosbridge_server rosbridge_suite rosbuild rosclean rosconsole rosconsole_bridge roscpp roscpp_serialization roscpp_traits roscreate rosgraph rosgraph_msgs roslang roslaunch roslib roslint roslisp roslz4 rosmake rosmaster rosmsg rosnode rosout rospack rosparam rospy rospy_tutorials rosserial rosserial_client rosserial_msgs rosserial_python rosservice rostest rostime rostopic rosunit roswtf sensor_msgs smclib std_msgs std_srvs stereo_msgs tf tf2 tf2_bullet tf2_eigen tf2_geometry_msgs tf2_kdl tf2_msgs tf2_py tf2_ros tf2_sensor_msgs tf2_tools topic_tools trajectory_msgs urdf urdf_parser_plugin usb_cam uuid_msgs visualization_msgs xmlrpcpp

#echo -e "\033[0;31m\033[1m$(date) | #12 Building heavy packages\033[0m\033[0m"

# This command uses less threads to avoid Raspberry Pi freeze
# Build the catkin Workspace
#cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -j1 --install-space /opt/ros/kinetic --pkg mavros opencv3 cv_bridge cv_camera mavros_extras web_video_server

echo -e "\033[0;31m\033[1m$(date) | #10 Building packages on 1 thread\033[0m\033[0m"

# Install builded packages
# WARNING: A major bug was found when using --pkg option (catkin_make_isolated doesn't install environment files)
# TODO: Can we increase threads number with HDD swap?
cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -j1 -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic

echo -e "\033[0;31m\033[1m$(date) | #11 Remove build_isolated & devel_isolated from ros_catkin_ws\033[0m\033[0m"

rm -rf /home/pi/ros_catkin_ws/build_isolated /home/pi/ros_catkin_ws/devel_isolated
chown -Rf pi:pi /home/pi/ros_catkin_ws

echo -e "\033[0;31m\033[1m$(date) | #12 Creating catkin_ws & Installing CLEVER-BUNDLE\033[0m\033[0m"

git clone $1 /home/pi/catkin_ws/src/clever \
  && pip install wheel \
  && pip install -r /home/pi/catkin_ws/src/clever/clever/requirements.txt \
  && cd /home/pi/catkin_ws \
  && . /opt/ros/kinetic/setup.sh \
  && catkin_make -j1 \
  && systemctl enable /home/pi/catkin_ws/src/clever/deploy/roscore.service \
  && systemctl enable /home/pi/catkin_ws/src/clever/deploy/clever.service

echo -e "\033[0;31m\033[1m$(date) | #13 Remove build dir from catkin_ws\033[0m\033[0m"

rm -rf /home/pi/catkin_ws/build
chown -Rf pi:pi /home/pi/catkin_ws

echo -e "\033[0;31m\033[1m$(date) | #14 Setup ROS environment\033[0m\033[0m"

cat <<EOF | tee -a /home/pi/.bashrc > /dev/null
LANG=C.UTF-8
LC_ALL=C.UTF-8
ROS_DISTRO=kinetic
export ROS_IP=192.168.11.1
source /opt/ros/kinetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
EOF

#echo -e "\033[0;31m\033[1m$(date) | #14 Removing local apt mirror\033[0m\033[0m"
# Restore original sources.list
#mv /var/sources.list.bak /etc/apt/sources.list
# Clean apt cache
apt-get clean
# Remove local mirror repository key
#apt-key del COEX-MIRROR

echo -e "\033[0;31m\033[1m$(date) | END of ROS INSTALLATION\033[0m\033[0m"
