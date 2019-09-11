#!/bin/bash

# Perform a "standalone install" in a Docker container

# Step 1: Install pip
apt update
apt install -y curl
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python ./get-pip.py

# Step 1.5: Add deb.coex.tech to apt
curl http://deb.coex.tech/aptly_repo_signing.key 2> /dev/null | apt-key add -
echo "deb http://deb.coex.tech/ros xenial main" > /etc/apt/sources.list.d/coex.tech.list
echo "yaml file:///etc/ros/rosdep/coex.yaml" > /etc/ros/rosdep/sources.list.d/99-coex.yaml
cat <<EOF > /etc/ros/rosdep/coex.yaml
led_msgs:
  ubuntu:
    xenial: ros-kinetic-led-msgs
    bionic: ros-melodic-led-msgs
  debian:
    stretch: ros-kinetic-led-msgs
    buster: ros-melodic-led-msgs
EOF
apt update
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
pip install --upgrade pytest
cd /root/catkin_ws
source devel/setup.bash
catkin_make run_tests && catkin_test_results
