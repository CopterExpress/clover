#! /usr/bin/env bash

set -e

cd ${IMAGE_BUILDER}

# Make free space
./image_config.sh resize_fs ${IMAGE_PATH} '7G'

./image_config.sh copy_to_chroot ${IMAGE_PATH} ${SCRIPTS_DIR}'/init_rpi.sh' '/root/'
./image_config.sh copy_to_chroot ${IMAGE_PATH} ${SCRIPTS_DIR}'/hardware_setup.sh' '/root/'

./image_config.sh execute ${IMAGE_PATH} ${SCRIPTS_DIR}'/init_image.sh' ${CLEVER_VERSION} $(jq '.source_image' -r ${TARGET_CONFIG})
./image_config.sh execute ${IMAGE_PATH} ${SCRIPTS_DIR}'/software_install.sh'
./image_config.sh execute ${IMAGE_PATH} ${SCRIPTS_DIR}'/network_setup.sh'

# If RPi then use a one thread to build a ROS package on RPi, else use all
[[ $(arch) == 'armv7l' ]] && NUMBER_THREADS=1 || NUMBER_THREADS=$(nproc --all)

./image_config.sh copy_to_chroot ${IMAGE_PATH} ${SCRIPTS_DIR}'/kinetic-ros-clever.rosinstall' '/home/pi/ros_catkin_ws/'
./image_config.sh execute ${IMAGE_PATH} ${SCRIPTS_DIR}'/ros_install.sh' ${TARGET_REPO} ${TARGET_REF} False False ${NUMBER_THREADS} 

./autosizer.sh ${IMAGE_PATH}
