#! /usr/bin/env bash

#
# Script for image configure
# @urpylka Artem Smirnov
#

# Exit immidiately on non-zero result
set -e

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

  TEXT="$(date) | $1"
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

# TODO: 'kinetic-rosdep-clever.yaml' should add only if we use our repo?
echo_stamp "Init rosdep" \
  && rosdep init \
  && echo "yaml file:///etc/ros/rosdep/kinetic-rosdep-clever.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list \
  && rosdep update

resolve_rosdep() {
  # TEMPLATE: resolve_rosdep <CATKIN_PATH> <ROS_DISTRO> <OS_DISTRO> <OS_VERSION>
  CATKIN_PATH=$1
  ROS_DISTRO='kinetic'
  OS_DISTRO='debian'
  OS_VERSION='stretch'

  echo_stamp "Installing dependencies apps with rosdep in ${CATKIN_PATH}"
  cd ${CATKIN_PATH}
  set +e
  # Successfull unmount flag (false at thismoment)
  install_ok=false
  # Repeat 5 times
  for i in {1..5}; do
    # Resolving Dependencies with rosdep
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r --os=${OS_DISTRO}:${OS_VERSION} \
    && (install_ok=true; break) || (echo_stamp "rosdep iteration #$i failed!" "ERROR"; sleep 2)
  done
  set -e
  # Stage fail if this condition is not true
  [[ $install_ok ]] \
  && echo_stamp "All rosdep dependencies was installed!" "SUCCESS" \
  || (echo_stamp "Rosdep installation was failed!" "ERROR"; exit 1)
}


INSTALL_ROS_PACK_SOURCES=${3:='False'}
if [ "${INSTALL_ROS_PACK_SOURCES}" = "True" ]; then
  DISCOVER_ROS_PACK=${4:='True'}
  if [ "${DISCOVER_ROS_PACK}" = "False" ]; then
    echo_stamp "Preparing ros_comm packages to kinetic-ros_comm-wet.rosinstall" \
      && mkdir -p /home/pi/ros_catkin_ws && cd /home/pi/ros_catkin_ws \
      && rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall \
      && wstool init -j$5 src kinetic-ros_comm-wet.rosinstall \
      && echo_stamp "All roscomm sources was installed!" "SUCCESS" \
      || (echo_stamp "Some roscomm sources installation was failed!" "ERROR"; exit 1)

    echo_stamp "Preparing other ROS-packages to kinetic-custom_ros.rosinstall" \
      && cd /home/pi/ros_catkin_ws \
      && rosinstall_generator \
      actionlib actionlib_msgs angles async_web_server_cpp bond bond_core bondcpp bondpy camera_calibration_parsers camera_info_manager catkin class_loader cmake_modules cpp_common cv_bridge cv_camera diagnostic_msgs diagnostic_updater dynamic_reconfigure eigen_conversions gencpp geneus genlisp genmsg gennodejs genpy geographic_msgs geometry_msgs geometry2 image_transport libmavconn mavlink mavros_msgs message_filters message_generation message_runtime mk nav_msgs nodelet orocos_kdl pluginlib python_orocos_kdl ros ros_comm rosapi rosauth rosbag rosbag_migration_rule rosbag_storage rosbash rosboost_cfg rosbridge_library rosbridge_server rosbridge_suite rosbuild rosclean rosconsole rosconsole_bridge roscpp roscpp_serialization roscpp_traits roscreate rosgraph rosgraph_msgs roslang roslaunch roslib roslint roslisp roslz4 rosmake rosmaster rosmsg rosnode rosout rospack rosparam rospy rospy_tutorials rosserial rosserial_client rosserial_msgs rosserial_python rosservice rostest rostime rostopic rosunit roswtf sensor_msgs smclib std_msgs std_srvs stereo_msgs tf tf2 tf2_bullet tf2_eigen tf2_geometry_msgs tf2_kdl tf2_msgs tf2_py tf2_ros tf2_sensor_msgs tf2_tools topic_tools trajectory_msgs urdf urdf_parser_plugin usb_cam uuid_msgs visualization_msgs web_video_server xmlrpcpp mavros opencv3 mavros_extras \
      --rosdistro kinetic --deps --wet-only --tar > kinetic-custom_ros.rosinstall \
      && wstool merge -j$5 -t src kinetic-custom_ros.rosinstall \
      && wstool update -j$5 -t src \
      && echo_stamp "All custom sources was installed!" "SUCCESS" \
      || (echo_stamp "Some custom sources installation was failed!" "ERROR"; exit 1)
  else
    echo_stamp "Creating ros_catkin_ws & getting all sources using wstool" \
      && mkdir -p /home/pi/ros_catkin_ws && cd /home/pi/ros_catkin_ws \
      && wstool init -j$5 src kinetic-ros-clever.rosinstall \
      > /dev/null \
      && echo_stamp "All CLEVER sources was installed!" "SUCCESS" \
      || (echo_stamp "Some CLEVER sources installation was failed!" "ERROR"; exit 1)
  fi

  resolve_rosdep '/home/pi/ros_catkin_ws'

  # TODO: Add refactor to origin repo
  #echo_stamp "Refactoring usb_cam in SRC"
  #sed -i '/#define __STDC_CONSTANT_MACROS/a\#define PIX_FMT_RGB24 AV_PIX_FMT_RGB24\n#define PIX_FMT_YUV422P AV_PIX_FMT_YUV422P' /home/pi/ros_catkin_ws/src/usb_cam/src/usb_cam.cpp

  # TODO: test without that
  #echo_stamp "#9 Installing GeographicLib datasets"
  #/home/pi/ros_catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

  echo_stamp "Building ros_catkin_ws packages"
  cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -j$5 -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic

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

echo_stamp "#12 Creating catkin_ws & Installing CLEVER-BUNDLE" \
  && git clone $1 /home/pi/catkin_ws/src/clever \
  && cd /home/pi/catkin_ws/src/clever \
  && git checkout $2 \
  && pip install wheel \
  && cd /home/pi/catkin_ws \
  && resolve_rosdep $(pwd) \
  && ls -l /opt/ros/kinetic \
  && source /opt/ros/kinetic/setup.bash \
  && catkin_make -j$5 -DCMAKE_BUILD_TYPE=Release \
  && ln -s /home/pi/catkin_ws/src/clever/deploy/roscore.service /lib/systemd/system/roscore.service \
  && ln -s /home/pi/catkin_ws/src/clever/deploy/clever.service /lib/systemd/system/clever.service \
  && systemctl enable roscore \
  && systemctl enable clever \
  && echo_stamp "All CLEVER-BUNDLE was installed!" "SUCCESS" \
  || (echo_stamp "CLEVER-BUNDLE installation was failed!" "ERROR"; exit 1)

echo_stamp "#13 Change permissions for catkin_ws"
chown -Rf pi:pi /home/pi/catkin_ws

echo_stamp "#14 Setup ROS environment"
cat << EOF >> /home/pi/.bashrc
LANG='C.UTF-8'
LC_ALL='C.UTF-8'
ROS_DISTRO='kinetic'
export ROS_IP='192.168.11.1'
source /opt/ros/kinetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
EOF

#echo_stamp "#16 Removing local apt mirror"
# Restore original sources.list
#mv /var/sources.list.bak /etc/apt/sources.list
# Clean apt cache
apt-get clean -qq > /dev/null
# Remove local mirror repository key
#apt-key del COEX-MIRROR

echo_stamp "END of ROS INSTALLATION"
