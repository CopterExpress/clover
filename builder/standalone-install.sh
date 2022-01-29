#!/bin/bash

# Perform a "standalone install" in a Docker container
set -e
# Step 1: Install pip
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 # https://github.com/osrf/docker_images/issues/535
apt-get update
apt-get install -y curl
if [ "x${ROS_PYTHON_VERSION}" = "x3" ]; then
  PYTHON=python3
  curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
else
  PYTHON=python
  curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py
fi
${PYTHON} ./get-pip.py

# Step 1.5: Add deb.coex.tech to apt
curl http://deb.coex.tech/aptly_repo_signing.key 2> /dev/null | apt-key add -
echo "deb http://deb.coex.tech/ros xenial main" > /etc/apt/sources.list.d/coex.tech.list
echo "yaml file:///etc/ros/rosdep/coex.yaml" > /etc/ros/rosdep/sources.list.d/99-coex.list
CODENAME=$(lsb_release -sc)

cat <<EOF > /etc/ros/rosdep/coex.yaml
led_msgs:
  ubuntu:
    ${CODENAME}: [ros-${ROS_DISTRO}-led-msgs]
async_web_server_cpp:
  ubuntu:
    ${CODENAME}: [ros-${ROS_DISTRO}-async-web-server-cpp]
ros_pytest:
  ubuntu:
    ${CODENAME}: [ros-${ROS_DISTRO}-ros-pytest]
tf2_web_republisher:
  ubuntu:
    ${CODENAME}: [ros-${ROS_DISTRO}-tf2-web-republisher]
web_video_server:
  ubuntu:
    ${CODENAME}: [ros-${ROS_DISTRO}-web-video-server]
ws281x:
  ubuntu:
    ${CODENAME}: [ros-${ROS_DISTRO}-ws281x]
EOF
apt-get update
rosdep update

# Step 2: Run rosdep to install all dependencies
cd /root/catkin_ws
rosdep install --from-paths src --ignore-src -y
# Step 2.5: mavros can't install its geographiclib dependencies
curl https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -o install_geographiclib_datasets.sh
chmod a+x ./install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

# Step 3: Build the packages
cd /root/catkin_ws
catkin_make

# Step 4: Run tests
${PYTHON} -m pip install --upgrade pytest
cd /root/catkin_ws
source devel/setup.bash
catkin_make run_tests && catkin_test_results

# Step 5: Install packages
catkin_make install
