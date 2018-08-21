#! /bin/bash

DEBIAN_FRONTEND=${DEBIAN_FRONTEND:='noninteractive'}
LANG=${LANG:='C.UTF-8'}
LC_ALL=${LC_ALL:='C.UTF-8'}

TARGET_REPO_URL=${TARGET_REPO_URL:='https://github.com/urpylka/clever.git'}
TARGET_REPO_REF=${TARGET_REPO_REF:='qemu_test_2'}
TARGET_REPO_PATH=${TARGET_REPO_PATH:='/image_builder'}

IMAGE_NAME=${IMAGE_NAME:='clever_docker.img'}
CLEVER_VERSION=${CLEVER_VERSION:='NO_VERSION'}

SOURCE_IMAGE=${SOURCE_IMAGE:='https://downloads.raspberrypi.org/raspbian_lite/images/raspbian_lite-2017-12-01/2017-11-29-raspbian-stretch-lite.zip'}
NUMBER_THREADS=${NUMBER_THREADS:=4}
DISCOVER_ROS_PACKAGES=${DISCOVER_ROS_PACKAGES:=False}
SHRINK=${SHRINK:=True}

set -e

apt-get update -qq > /dev/null
apt-get install -y --no-install-recommends -qq unzip wget parted apt-utils git ca-certificates > /dev/null

REPO_DIR=$(mktemp -d)
git clone ${TARGET_REPO_URL} --single-branch --branch ${TARGET_REPO_REF} --depth 1 ${REPO_DIR}
[[ $? != 0 ]] && (echo 'Error: Could not clone repo!'; return 1)
[[ -d ${REPO_DIR}${TARGET_REPO_PATH} ]] || (echo 'Error: TARGET_REPO_PATH was incorrect!'; return 1)

BUILD_DIR=$(pwd)/image

./image_config.sh get_image ${BUILD_DIR} ${SOURCE_IMAGE} ${IMAGE_NAME}
./image_config.sh resize_fs ${BUILD_DIR}/${IMAGE_NAME} '7G'

if [[ $(arch) != 'armv7l' ]]; then
  if ! [[ -d '/proc/sys/fs/binfmt_misc' ]]; then
    mount binfmt_misc -t binfmt_misc /proc/sys/fs/binfmt_misc
    echo ':arm:M::\x7fELF\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x28\x00:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/bin/qemu-arm-static:' > /proc/sys/fs/binfmt_misc/register
  fi
fi
./image_config.sh copy_to_chroot ${BUILD_DIR}/${IMAGE_NAME} './qemu-arm-resin' '/usr/bin/qemu-arm-static'

./image_config.sh copy_to_chroot ${BUILD_DIR}/${IMAGE_NAME} './scripts/init_rpi.sh' '/root/'
./image_config.sh copy_to_chroot ${BUILD_DIR}/${IMAGE_NAME} './scripts/hardware_setup.sh' '/root/'

./image_config.sh execute ${BUILD_DIR}/${IMAGE_NAME} './scripts/init_image.sh' ${CLEVER_VERSION} ${SOURCE_IMAGE}
./image_config.sh execute ${BUILD_DIR}/${IMAGE_NAME} './scripts/software_install.sh'
./image_config.sh execute ${BUILD_DIR}/${IMAGE_NAME} './scripts/network_setup.sh'

#./image_config.sh copy_to_chroot ${BUILD_DIR}/${IMAGE_NAME} 'kinetic-ros-coex.rosinstall' '/home/pi/ros_catkin_ws/'

#
# Не совсем корректно тк такого параметра быть не должно,
# вообще здесь не должно быть ветки как параметра, она должна задаваться непосредственно в файл ros_install
#
#./image_config.sh execute ${BUILD_DIR}/${IMAGE_NAME} './scripts/ros_install.sh' https://github.com/CopterExpress/clever.git master True

if SHRINK; then
  ./autosizer.sh ${BUILD_DIR}/${IMAGE_NAME}
  # Наверное вставить в autosizer, тк отдельно (внутри образа) это все равно никто не будет запускать
  # и не нужно его запускать через execute
  ./image_config.sh execute ${BUILD_DIR}/${IMAGE_NAME} './scripts/change_boot_part.sh'
fi