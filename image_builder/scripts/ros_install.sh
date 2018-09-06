#!/bin/bash

##################################################################################################################################
# ROS for user pi
##################################################################################################################################

# ros http://wiki.ros.org/action/fullsearch/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi

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

echo_stamp "Installing ROS"

echo_stamp "#1 Installing dirmngr & add key to apt-key"
apt-get install --no-install-recommends -y -qq dirmngr=2.1.18-8~deb9u2 > /dev/null
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
echo "deb http://packages.ros.org/ros/ubuntu stretch main" > /etc/apt/sources.list.d/ros-latest.list

echo_stamp "#2 apt update && apt upgrade"
apt-get update -qq > /dev/null

echo_stamp "#3 Installing wget, unzip, python-rosdep, python-rosinstall-generator, python-wstool, python-rosinstall, build-essential, cmake"

apt-get install --no-install-recommends -y -qq \
  python-rosdep=0.12.2-1 \
  python-rosinstall-generator=0.1.14-1 \
  python-wstool=0.1.17-1 \
  python-rosinstall=0.7.8-1 \
  build-essential=12.3 \
  > /dev/null

echo_stamp "#4 rosdep init && rosdep update"
rosdep init && rosdep update

# If $3 = false, then discover packages
if [ "$3" = "false" ];
then
  echo_stamp "#5 Preparing ros_comm packages to kinetic-ros_comm-wet.rosinstall"

  # create ros catkin workspace
  mkdir -p /home/pi/ros_catkin_ws && cd /home/pi/ros_catkin_ws \
      && rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall \
      && wstool init src kinetic-ros_comm-wet.rosinstall

  echo_stamp "#6 Preparing other ROS-packages to kinetic-custom_ros.rosinstall"

  cd /home/pi/ros_catkin_ws \
    && rosinstall_generator \
    actionlib actionlib_msgs angles async_web_server_cpp bond bond_core bondcpp bondpy camera_calibration_parsers camera_info_manager catkin class_loader cmake_modules cpp_common cv_bridge cv_camera diagnostic_msgs diagnostic_updater dynamic_reconfigure eigen_conversions gencpp geneus genlisp genmsg gennodejs genpy geographic_msgs geometry_msgs geometry2 image_transport libmavconn mavlink mavros_msgs message_filters message_generation message_runtime mk nav_msgs nodelet orocos_kdl pluginlib python_orocos_kdl ros ros_comm rosapi rosauth rosbag rosbag_migration_rule rosbag_storage rosbash rosboost_cfg rosbridge_library rosbridge_server rosbridge_suite rosbuild rosclean rosconsole rosconsole_bridge roscpp roscpp_serialization roscpp_traits roscreate rosgraph rosgraph_msgs roslang roslaunch roslib roslint roslisp roslz4 rosmake rosmaster rosmsg rosnode rosout rospack rosparam rospy rospy_tutorials rosserial rosserial_client rosserial_msgs rosserial_python rosservice rostest rostime rostopic rosunit roswtf sensor_msgs smclib std_msgs std_srvs stereo_msgs tf tf2 tf2_bullet tf2_eigen tf2_geometry_msgs tf2_kdl tf2_msgs tf2_py tf2_ros tf2_sensor_msgs tf2_tools topic_tools trajectory_msgs urdf urdf_parser_plugin usb_cam uuid_msgs visualization_msgs web_video_server xmlrpcpp mavros opencv3 mavros_extras \
    --rosdistro kinetic --deps --wet-only --tar > kinetic-custom_ros.rosinstall \
    && wstool merge -t src kinetic-custom_ros.rosinstall \
    && wstool update -t src
else
  echo_stamp "#5 Creating ros_catkin_ws & getting all sources using wstool"
  mkdir -p /home/pi/ros_catkin_ws && cd /home/pi/ros_catkin_ws \
  && wstool init src kinetic-ros-coex.rosinstall \
  > /dev/null \
  && echo_stamp "ros_catkin_ws was created!" "SUCCESS" \
  || (echo_stamp "ros_catkin_ws wasn't created!" "ERROR"; exit 1)
fi

echo_stamp "#7 Installing dependencies apps with rosdep"
cd /home/pi/ros_catkin_ws
set +e
# Successfull unmount flag (false at thismoment)
install_ok=false
# Repeat 5 times
for i in {1..5}; do
  # Resolving Dependencies with rosdep
  rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:stretch \
  > /dev/null \
  && (install_ok=true; break) || (echo_stamp "rosdep iteration #$i failed!" "ERROR"; sleep 2)
done
set -e
# Stage fail if this condition is not true
[[ $install_ok ]] \
&& echo_stamp "All rosdep dependencies was installed!" "SUCCESS" \
|| (echo_stamp "Rosdep installation failed!" "ERROR"; exit 1)

echo_stamp "#8 Refactoring usb_cam in SRC"
sed -i '/#define __STDC_CONSTANT_MACROS/a\#define PIX_FMT_RGB24 AV_PIX_FMT_RGB24\n#define PIX_FMT_YUV422P AV_PIX_FMT_YUV422P' /home/pi/ros_catkin_ws/src/usb_cam/src/usb_cam.cpp

echo_stamp "#9 Installing GeographicLib datasets"
/home/pi/ros_catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

#echo_stamp "#11 Building light packages on 2 threads"

# Build the catkin Workspace
#cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -j2 --install-space /opt/ros/kinetic --pkg actionlib actionlib_msgs angles async_web_server_cpp bond bond_core bondcpp bondpy camera_calibration_parsers camera_info_manager catkin class_loader cmake_modules cpp_common diagnostic_msgs diagnostic_updater dynamic_reconfigure eigen_conversions gencpp geneus genlisp genmsg gennodejs genpy geographic_msgs geometry_msgs geometry2 image_transport libmavconn mavlink mavros_msgs message_filters message_generation message_runtime mk nav_msgs nodelet orocos_kdl pluginlib python_orocos_kdl ros ros_comm rosapi rosauth rosbag rosbag_migration_rule rosbag_storage rosbash rosboost_cfg rosbridge_library rosbridge_server rosbridge_suite rosbuild rosclean rosconsole rosconsole_bridge roscpp roscpp_serialization roscpp_traits roscreate rosgraph rosgraph_msgs roslang roslaunch roslib roslint roslisp roslz4 rosmake rosmaster rosmsg rosnode rosout rospack rosparam rospy rospy_tutorials rosserial rosserial_client rosserial_msgs rosserial_python rosservice rostest rostime rostopic rosunit roswtf sensor_msgs smclib std_msgs std_srvs stereo_msgs tf tf2 tf2_bullet tf2_eigen tf2_geometry_msgs tf2_kdl tf2_msgs tf2_py tf2_ros tf2_sensor_msgs tf2_tools topic_tools trajectory_msgs urdf urdf_parser_plugin usb_cam uuid_msgs visualization_msgs xmlrpcpp

#echo_stamp "#12 Building heavy packages"

# This command uses less threads to avoid Raspberry Pi freeze
# Build the catkin Workspace
#cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -j1 --install-space /opt/ros/kinetic --pkg mavros opencv3 cv_bridge cv_camera mavros_extras web_video_server

echo_stamp "#10 Building packages"

# Install builded packages
# WARNING: A major bug was found when using --pkg option (catkin_make_isolated doesn't install environment files)
# TODO: Can we increase threads number with HDD swap?
cd /home/pi/ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install -j$4 -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic

echo_stamp "#11 Remove build_isolated & devel_isolated from ros_catkin_ws"
rm -rf /home/pi/ros_catkin_ws/build_isolated /home/pi/ros_catkin_ws/devel_isolated
chown -Rf pi:pi /home/pi/ros_catkin_ws

echo_stamp "#12 Creating catkin_ws & Installing CLEVER-BUNDLE"
git clone $1 /home/pi/catkin_ws/src/clever \
  && cd /home/pi/catkin_ws/src/clever \
  && git checkout $2 \
  && pip install wheel \
  && pip install -r /home/pi/catkin_ws/src/clever/clever/requirements.txt \
  && cd /home/pi/catkin_ws \
  && . /opt/ros/kinetic/setup.sh \
  && catkin_make -j$4 -DCMAKE_BUILD_TYPE=Release \
  && ln -s /home/pi/catkin_ws/src/clever/deploy/roscore.service /lib/systemd/system/roscore.service \
  && ln -s /home/pi/catkin_ws/src/clever/deploy/clever.service /lib/systemd/system/clever.service \
  && systemctl enable roscore \
  && systemctl enable clever

echo_stamp "#13 Change permissions for catkin_ws"
chown -Rf pi:pi /home/pi/catkin_ws

echo_stamp "#14 Setup ROS environment"
cat <<EOF | tee -a /home/pi/.bashrc > /dev/null
LANG=C.UTF-8
LC_ALL=C.UTF-8
ROS_DISTRO=kinetic
export ROS_IP=192.168.11.1
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
