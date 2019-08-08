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

set -e # Exit immidiately on non-zero result

REPO=$1
REF=$2
INSTALL_ROS_PACK_SOURCES=$3
DISCOVER_ROS_PACK=$4
NUMBER_THREADS=$5

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

  TEXT="$(date '+[%Y-%m-%d %H:%M:%S]') $1"
  TEXT="\e[1m$TEXT\e[0m" # BOLD

  case "$2" in
    SUCCESS)
    TEXT="\e[32m${TEXT}\e[0m";; # GREEN
    ERROR)
    TEXT="\e[31m${TEXT}\e[0m";; # RED
    *)
    TEXT="\e[34m${TEXT}\e[0m";; # BLUE
  esac
  echo -e ${TEXT}
}

# https://gist.github.com/letmaik/caa0f6cc4375cbfcc1ff26bd4530c2a3
# https://github.com/travis-ci/travis-build/blob/master/lib/travis/build/templates/header.sh
my_travis_retry() {
  local result=0
  local count=1
  local max_count=50
  while [ $count -le $max_count ]; do
    [ $result -ne 0 ] && {
      echo -e "\nThe command \"$@\" failed. Retrying, $count of $max_count.\n" >&2
    }
    # ! { } ignores set -e, see https://stackoverflow.com/a/4073372
    ! { "$@"; result=$?; }
    [ $result -eq 0 ] && break
    count=$(($count + 1))
    sleep 1
  done

  [ $count -gt $max_count ] && {
    echo -e "\nThe command \"$@\" failed $max_count times.\n" >&2
  }

  return $result
}

# TODO: 'kinetic-rosdep-clever.yaml' should add only if we use our repo?
echo_stamp "Init rosdep"
my_travis_retry rosdep init
echo "yaml file:///etc/ros/rosdep/kinetic-rosdep-clever.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list
my_travis_retry rosdep update

echo_stamp "Populate rosdep for ROS user"
my_travis_retry sudo -u pi rosdep update

resolve_rosdep() {
  # TEMPLATE: resolve_rosdep <CATKIN_PATH> <ROS_DISTRO> <OS_DISTRO> <OS_VERSION>
  CATKIN_PATH=$1
  ROS_DISTRO='kinetic'
  OS_DISTRO='debian'
  OS_VERSION='buster'

  echo_stamp "Installing dependencies apps with rosdep in ${CATKIN_PATH}"
  cd ${CATKIN_PATH}
  my_travis_retry rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --os=${OS_DISTRO}:${OS_VERSION}
}

INSTALL_ROS_PACK_SOURCES=${INSTALL_ROS_PACK_SOURCES:='false'}
if [ "${INSTALL_ROS_PACK_SOURCES}" = "true" ]; then
  DISCOVER_ROS_PACK=${DISCOVER_ROS_PACK:='true'}
  if [ "${DISCOVER_ROS_PACK}" = "false" ]; then
    echo_stamp "Preparing ros_comm packages to kinetic-ros_comm-wet.rosinstall" \
    && mkdir -p /home/pi/ros_catkin_ws && cd /home/pi/ros_catkin_ws \
    && rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall \
    && wstool init -j${NUMBER_THREADS} src kinetic-ros_comm-wet.rosinstall \
    && echo_stamp "All roscomm sources was installed!" "SUCCESS" \
    || (echo_stamp "Some roscomm sources installation was failed!" "ERROR"; exit 1)

    echo_stamp "Preparing other ROS-packages to kinetic-custom_ros.rosinstall" \
    && cd /home/pi/ros_catkin_ws \
    && rosinstall_generator \
    actionlib actionlib_msgs angles async_web_server_cpp bond bond_core bondcpp bondpy camera_calibration_parsers camera_info_manager catkin class_loader cmake_modules cpp_common cv_bridge cv_camera diagnostic_msgs diagnostic_updater dynamic_reconfigure eigen_conversions gencpp geneus genlisp genmsg gennodejs genpy geographic_msgs geometry_msgs geometry2 image_transport compressed_image_transport libmavconn mavlink mavros_msgs message_filters message_generation message_runtime mk nav_msgs nodelet orocos_kdl pluginlib python_orocos_kdl ros ros_comm rosapi rosauth rosbag rosbag_migration_rule rosbag_storage rosbash rosboost_cfg rosbridge_library rosbridge_server rosbridge_suite rosbuild rosclean rosconsole rosconsole_bridge roscpp roscpp_serialization roscpp_traits roscreate rosgraph rosgraph_msgs roslang roslaunch roslib roslint roslisp roslz4 rosmake rosmaster rosmsg rosnode rosout rospack rosparam rospy rospy_tutorials rosserial rosserial_client rosserial_msgs rosserial_python rosservice rostest rostime rostopic rosunit roswtf sensor_msgs smclib std_msgs std_srvs stereo_msgs tf tf2 tf2_bullet tf2_eigen tf2_geometry_msgs tf2_kdl tf2_msgs tf2_py tf2_ros tf2_sensor_msgs tf2_tools topic_tools trajectory_msgs urdf urdf_parser_plugin usb_cam uuid_msgs visualization_msgs web_video_server xmlrpcpp mavros opencv3 mavros_extras interactive_markers tf2_web_republisher interactive_marker_proxy vl53l1x \
    --rosdistro kinetic --deps --wet-only --tar > kinetic-custom_ros.rosinstall \
    && wstool merge -j${NUMBER_THREADS} -t src kinetic-custom_ros.rosinstall \
    && wstool update -j${NUMBER_THREADS} -t src \
    && echo_stamp "All custom sources was installed!" "SUCCESS" \
    || (echo_stamp "Some custom sources installation was failed!" "ERROR"; exit 1)
  else
    echo_stamp "Creating ros_catkin_ws & getting all sources using wstool" \
    && mkdir -p /home/pi/ros_catkin_ws && cd /home/pi/ros_catkin_ws \
    && wstool init -j${NUMBER_THREADS} src kinetic-ros-clever.rosinstall \
    > /dev/null \
    && echo_stamp "All CLEVER sources was installed!" "SUCCESS" \
    || (echo_stamp "Some CLEVER sources installation was failed!" "ERROR"; exit 1)
  fi

  resolve_rosdep '/home/pi/ros_catkin_ws'

  # TODO: Add refactor to origin repo
  #echo_stamp "Refactoring usb_cam in SRC"
  #sed -i '/#define __STDC_CONSTANT_MACROS/a\#define PIX_FMT_RGB24 AV_PIX_FMT_RGB24\n#define PIX_FMT_YUV422P AV_PIX_FMT_YUV422P' /home/pi/ros_catkin_ws/src/usb_cam/src/usb_cam.cpp

  echo_stamp "Building ros_catkin_ws packages"
  cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -j${NUMBER_THREADS} -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic

  #echo_stamp "#11 Building light packages on 2 threads"
  #cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -j2 --install-space /opt/ros/kinetic --pkg actionlib actionlib_msgs angles async_web_server_cpp bond bond_core bondcpp bondpy camera_calibration_parsers camera_info_manager catkin class_loader cmake_modules cpp_common diagnostic_msgs diagnostic_updater dynamic_reconfigure eigen_conversions gencpp geneus genlisp genmsg gennodejs genpy geographic_msgs geometry_msgs geometry2 image_transport libmavconn mavlink mavros_msgs message_filters message_generation message_runtime mk nav_msgs nodelet orocos_kdl pluginlib python_orocos_kdl ros ros_comm rosapi rosauth rosbag rosbag_migration_rule rosbag_storage rosbash rosboost_cfg rosbridge_library rosbridge_server rosbridge_suite rosbuild rosclean rosconsole rosconsole_bridge roscpp roscpp_serialization roscpp_traits roscreate rosgraph rosgraph_msgs roslang roslaunch roslib roslint roslisp roslz4 rosmake rosmaster rosmsg rosnode rosout rospack rosparam rospy rospy_tutorials rosserial rosserial_client rosserial_msgs rosserial_python rosservice rostest rostime rostopic rosunit roswtf sensor_msgs smclib std_msgs std_srvs stereo_msgs tf tf2 tf2_bullet tf2_eigen tf2_geometry_msgs tf2_kdl tf2_msgs tf2_py tf2_ros tf2_sensor_msgs tf2_tools topic_tools trajectory_msgs urdf urdf_parser_plugin usb_cam uuid_msgs visualization_msgs xmlrpcpp

  #echo_stamp "#12 Building heavy packages"
  # This command uses less threads to avoid Raspberry Pi freeze
  #cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -j1 --install-space /opt/ros/kinetic --pkg mavros opencv3 cv_bridge cv_camera mavros_extras web_video_server

  # Install builded packages
  # WARNING: A major bug was found when using --pkg option (catkin_make_isolated doesn't install environment files)
  # TODO: Can we increase threads number with HDD swap?

  echo_stamp "Remove build_isolated & devel_isolated from ros_catkin_ws"
  rm -rf /home/pi/ros_catkin_ws/build_isolated /home/pi/ros_catkin_ws/devel_isolated
  chown -Rf pi:pi /home/pi/ros_catkin_ws
fi

export ROS_IP='127.0.0.1' # needed for running tests

echo_stamp "Reconfiguring Clever repository for simplier unshallowing"
cd /home/pi/catkin_ws/src/clever
git config remote.origin.fetch "+refs/heads/*:refs/remotes/origin/*"

echo_stamp "Installing CLEVER" \
&& cd /home/pi/catkin_ws/src/clever \
&& git status \
&& cd /home/pi/catkin_ws \
&& resolve_rosdep $(pwd) \
&& my_travis_retry pip install wheel \
&& my_travis_retry pip install -r /home/pi/catkin_ws/src/clever/clever/requirements.txt \
&& source /opt/ros/kinetic/setup.bash \
&& catkin_make -j2 -DCMAKE_BUILD_TYPE=Release \
&& systemctl enable roscore \
&& systemctl enable clever \
&& echo_stamp "All CLEVER was installed!" "SUCCESS" \
|| (echo_stamp "CLEVER installation was failed!" "ERROR"; exit 1)

echo_stamp "Build CLEVER documentation"
cd /home/pi/catkin_ws/src/clever
NPM_CONFIG_UNSAFE_PERM=true npm install gitbook-cli -g
NPM_CONFIG_UNSAFE_PERM=true gitbook install
gitbook build

echo_stamp "Installing additional ROS packages"
apt-get install -y --no-install-recommends \
    ros-kinetic-dynamic-reconfigure \
    ros-kinetic-compressed-image-transport \
    ros-kinetic-rosbridge-suite \
    ros-kinetic-rosserial \
    ros-kinetic-usb-cam \
    ros-kinetic-vl53l1x \
    ros-kinetic-ws281x \
    ros-kinetic-rosshow

# TODO move GeographicLib datasets to Mavros debian package
echo_stamp "Install GeographicLib datasets (needed for mavros)" \
&& wget -qO- https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash

echo_stamp "Running tests"
catkin_make run_tests && catkin_test_results

echo_stamp "Change permissions for catkin_ws"
chown -Rf pi:pi /home/pi/catkin_ws

echo_stamp "Setup ROS environment"
cat << EOF >> /home/pi/.bashrc
LANG='C.UTF-8'
LC_ALL='C.UTF-8'
ROS_DISTRO='kinetic'
export ROS_HOSTNAME=\`hostname\`.local
source /opt/ros/kinetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
EOF

#echo_stamp "Removing local apt mirror"
# Restore original sources.list
#mv /var/sources.list.bak /etc/apt/sources.list
# Clean apt cache
apt-get clean -qq > /dev/null
# Remove local mirror repository key
#apt-key del COEX-MIRROR

echo_stamp "END of ROS INSTALLATION"
