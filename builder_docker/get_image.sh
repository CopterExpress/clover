#! /usr/bin/env bash

#
# Script for image configure
# @urpylka Artem Smirnov
# @dvornikov-aa Andrey Dvornikov
#

# Exit immidiately on non-zero result
set -e

source echo_stamp.sh

get_image() {
  # TEMPLATE: get_image <IMAGE_PATH> <RPI_DONWLOAD_URL> 
  local BUILD_DIR=$(dirname $1)
  local RPI_ZIP_NAME=$(basename $2)
  if [ ! -e "${BUILD_DIR}/${RPI_ZIP_NAME}" ];
  then
    echo_stamp "1. Downloading original Linux distribution"
    wget -nv -O ${BUILD_DIR}/${RPI_ZIP_NAME} $2 \
    && echo_stamp "Downloading complete" "SUCCESS"
  else
    echo_stamp "1. Linux distribution already donwloaded"
  fi
  echo_stamp "2. Unzipping Linux distribution image"
  local RPI_IMAGE_NAME=$(echo ${RPI_ZIP_NAME} | sed 's/zip/img/')
  unzip -p ${BUILD_DIR}/${RPI_ZIP_NAME} ${RPI_IMAGE_NAME} > $1
  echo_stamp "Unzipping complete" "SUCCESS"
}

get_image $1 $2
