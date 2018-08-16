#! /bin/sh

DEBIAN_FRONTEND = 'noninteractive'
LANG = 'C.UTF-8'
LC_ALL = 'C.UTF-8'

WORKSPACE=$(pwd)
BUILD_DIR=$(pwd)

set -e

BUILD_DIR="/mnt/hdd_builder/workspace"
RPI_DONWLOAD_URL="https://downloads.raspberrypi.org/raspbian_lite/images/raspbian_lite-2017-12-01/2017-11-29-raspbian-stretch-lite.zip"
IMAGE_NAME="clever_qemu.img"

./image_config.sh get_image ${BUILD_DIR} ${RPI_DONWLOAD_URL} ${IMAGE_NAME}

MOVE_FILE="qemu-arm-orig"
MOVE_TO="/usr/bin/qemu-arm-static"

echo -e "\033[0;31m\033[1mAdd qemu-interpretator\033[0m\033[0m"
./image_config.sh copy_to_chroot ${BUILD_DIR}/${IMAGE_NAME} $WORKSPACE/${MOVE_FILE} ${MOVE_TO}

#if ! [ -f $2/usr/bin/qemu-arm-static ];
#then cp $6/image/qemu-arm-orig $2/usr/bin/qemu-arm-static
#fi

EXECUTE_FILE=$WORKSPACE/scripts/hardware_setup.sh

./image_config.sh execute ${params.BUILD_DIR}/${params.IMAGE_NAME} $WORKSPACE/$EXECUTE_FILE