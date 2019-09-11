#!/bin/bash

# Perform a "standalone install" in a Docker container

# Step 1: Install pip
apt update
apt install -y curl
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python ./get-pip.py

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
