#! /bin/bash

DEBIAN_FRONTEND='noninteractive'
LANG='C.UTF-8'
LC_ALL='C.UTF-8'

mount binfmt_misc -t binfmt_misc /proc/sys/fs/binfmt_misc
echo ':arm:M::\x7fELF\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x28\x00:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/bin/qemu-arm-static:' > /proc/sys/fs/binfmt_misc/register

apt install unzip wget

BUILD_DIR=$(pwd)

set -e

RPI_DONWLOAD_URL="https://downloads.raspberrypi.org/raspbian_lite/images/raspbian_lite-2017-12-01/2017-11-29-raspbian-stretch-lite.zip"
IMAGE_NAME="clever_qemu.img"

./image_config.sh get_image ${BUILD_DIR} ${RPI_DONWLOAD_URL} ${IMAGE_NAME}
./image_config.sh resize_fs ${BUILD_DIR}/${IMAGE_NAME} 7G

./image_config.sh copy_to_chroot ${BUILD_DIR}/${IMAGE_NAME} "qemu-arm-resin" "/usr/bin/qemu-arm-static"
./image_config.sh copy_to_chroot ${BUILD_DIR}/${IMAGE_NAME} "scripts" "/"
./image_config.sh copy_to_chroot ${BUILD_DIR}/${IMAGE_NAME} "kinetic-ros-coex.rosinstall" "/home/pi/ros_catkin_ws/"

#if ! [ -f $2/usr/bin/qemu-arm-static ];
#then cp $6/image/qemu-arm-orig $2/usr/bin/qemu-arm-static
#fi

./image_config.sh execute ${BUILD_DIR}/${IMAGE_NAME} scripts/change_boot_part.sh
./image_config.sh execute ${BUILD_DIR}/${IMAGE_NAME} scripts/init_image.sh qemu_build_17082018 raspbian_nov_2017
./image_config.sh execute ${BUILD_DIR}/${IMAGE_NAME} scripts/software_install.sh
./image_config.sh execute ${BUILD_DIR}/${IMAGE_NAME} scripts/network_setup.sh
./image_config.sh execute ${BUILD_DIR}/${IMAGE_NAME} scripts/ros_install.sh https://github.com/CopterExpress/clever.git master True
