#! /usr/bin/env bash

#
# Script for build the image. Used builder script of the target repo
# For build: docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/builder/repo smirart/builder
#
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Artem Smirnov <urpylka@gmail.com>
#

set -e # Exit immidiately on non-zero result

SOURCE_IMAGE="https://downloads.raspberrypi.org/raspbian_lite/images/raspbian_lite-2018-06-29/2018-06-27-raspbian-stretch-lite.zip"

export DEBIAN_FRONTEND=${DEBIAN_FRONTEND:='noninteractive'}
export LANG=${LANG:='C.UTF-8'}
export LC_ALL=${LC_ALL:='C.UTF-8'}

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

BUILDER_DIR="/builder"
REPO_DIR="${BUILDER_DIR}/repo"
SCRIPTS_DIR="${REPO_DIR}/builder"
IMAGES_DIR="${REPO_DIR}/images"

[[ ! -d ${SCRIPTS_DIR} ]] && (echo_stamp "Directory ${SCRIPTS_DIR} doesn't exist" "ERROR"; exit 1)
[[ ! -d ${IMAGES_DIR} ]] && mkdir ${IMAGES_DIR} && echo_stamp "Directory ${IMAGES_DIR} was created successful" "SUCCESS"

if [[ -z ${TRAVIS_TAG} ]]; then IMAGE_VERSION="$(cd ${REPO_DIR}; git log --format=%h -1)"; else IMAGE_VERSION="${TRAVIS_TAG}"; fi
# IMAGE_VERSION="${TRAVIS_TAG:=$(cd ${REPO_DIR}; git log --format=%h -1)}"
REPO_URL="$(cd ${REPO_DIR}; git remote --verbose | grep origin | grep fetch | cut -f2 | cut -d' ' -f1 | sed 's/git@github\.com\:/https\:\/\/github.com\//')"
REPO_NAME="$(basename -s '.git' ${REPO_URL})"
IMAGE_NAME="${REPO_NAME}_${IMAGE_VERSION}.img"
IMAGE_PATH="${IMAGES_DIR}/${IMAGE_NAME}"

get_image() {
  # TEMPLATE: get_image <IMAGE_PATH> <RPI_DONWLOAD_URL>
  local BUILD_DIR=$(dirname $1)
  local RPI_ZIP_NAME=$(basename $2)
  local RPI_IMAGE_NAME=$(echo ${RPI_ZIP_NAME} | sed 's/zip/img/')

  if [ ! -e "${BUILD_DIR}/${RPI_ZIP_NAME}" ]; then
    echo_stamp "Downloading original Linux distribution"
    wget --progress=dot:giga -O ${BUILD_DIR}/${RPI_ZIP_NAME} $2
    echo_stamp "Downloading complete" "SUCCESS" \
  else echo_stamp "Linux distribution already donwloaded"; fi

  echo_stamp "Unzipping Linux distribution image" \
  && unzip -p ${BUILD_DIR}/${RPI_ZIP_NAME} ${RPI_IMAGE_NAME} > $1 \
  && echo_stamp "Unzipping complete" "SUCCESS" \
  || (echo_stamp "Unzipping was failed!" "ERROR"; exit 1)
}

get_image ${IMAGE_PATH} ${SOURCE_IMAGE}

# Make free space
${BUILDER_DIR}/image-resize.sh ${IMAGE_PATH} max '7G'

${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/init_rpi.sh' '/root/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/hardware_setup.sh' '/root/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec ${SCRIPTS_DIR}'/image-init.sh' ${IMAGE_VERSION} ${SOURCE_IMAGE}

# Copy cloned repository to the image
# Include dotfiles in globs (asterisks)
shopt -s dotglob
for dir in ${REPO_DIR}/*; do
  # Don't try to copy image into itself
  if [[ $dir != *"images" && $dir != *"imgcache" ]]; then
    ${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy $dir '/home/pi/catkin_ws/src/clever/'
  fi;
done

# Monkey
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/monkey' '/root/'

# Butterfly
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/butterfly.service' '/lib/systemd/system/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/butterfly.socket' '/lib/systemd/system/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/monkey.service' '/lib/systemd/system/'
# software install
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec ${SCRIPTS_DIR}'/image-software.sh'
# network setup
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec ${SCRIPTS_DIR}'/image-network.sh'

# If RPi then use a one thread to build a ROS package on RPi, else use all
[[ $(arch) == 'armv7l' ]] && NUMBER_THREADS=1 || NUMBER_THREADS=$(nproc --all)
# Clever
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/clever.service' '/lib/systemd/system/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/roscore.env' '/lib/systemd/system/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/roscore.service' '/lib/systemd/system/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/kinetic-rosdep-clever.yaml' '/etc/ros/rosdep/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/ros_python_paths' '/etc/sudoers.d/'
# ${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/kinetic-ros-clever.rosinstall' '/home/pi/ros_catkin_ws/'
# Add PX4 udev rules
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/99-px4fmu.rules' '/lib/udev/rules.d/'
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec ${SCRIPTS_DIR}'/image-ros.sh' ${REPO_URL} ${IMAGE_VERSION} false false ${NUMBER_THREADS}
${BUILDER_DIR}/image-chroot.sh ${IMAGE_PATH} exec ${SCRIPTS_DIR}'/image-validate.sh'

${BUILDER_DIR}/image-resize.sh ${IMAGE_PATH}
