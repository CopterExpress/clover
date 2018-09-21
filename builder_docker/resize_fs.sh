#! /usr/bin/env bash

#
# Script for image configure
# @urpylka Artem Smirnov
# @dvornikov-aa Andrey Dvornikov
#

# Exit immidiately on non-zero result
set -e

source echo_stamp.sh

resize_fs() {
  # TEMPLATE: resize_fs <IMAGE_PATH> <SIZE>

  set +e

  # https://ru.wikipedia.org/wiki/%D0%A0%D0%B0%D0%B7%D1%80%D0%B5%D0%B6%D1%91%D0%BD%D0%BD%D1%8B%D0%B9_%D1%84%D0%B0%D0%B9%D0%BB

  # https://raspberrypi.stackexchange.com/questions/13137/how-can-i-mount-a-raspberry-pi-linux-distro-image
  # fdisk -l 2017-11-29-raspbian-stretch-lite.img
  # https://www.stableit.ru/2011/05/losetup.html
  # -f     : losetup сам выбрал loop (минуя занятые)
  # -P     : losetup монтирует разделы в образе как отдельные подразделы,
  #          например /dev/loop0p1 и /dev/loop0p2
  # --show : печатает имя устройства, например /dev/loop4

  # http://karelzak.blogspot.ru/2015/05/resize-by-sfdisk.html
  # ", +" : expand partition for volume size
  # -N 2  : select second partition for work

  # There is a risk that sfdisk will ask for a disk remount to update partition table
  # TODO: Check sfdisk exit code

  echo_stamp "Truncate image" \
  && truncate -s$2 $1 \
  && echo_stamp "Mount loop-image: $1" \
  && local DEV_IMAGE=$(losetup -Pf $1 --show) \
  && sleep 0.5 \
  && echo ", +" | sfdisk -N 2 ${DEV_IMAGE} \
  && sleep 0.5 \
  && losetup -d ${DEV_IMAGE} \
  && sleep 0.5 \
  && local DEV_IMAGE=$(losetup -Pf $1 --show) \
  && sleep 0.5 \
  && echo_stamp "Check & repair filesystem after expand partition" \
  && e2fsck -fvy "${DEV_IMAGE}p2" \
  && echo_stamp "Expand filesystem" \
  && resize2fs "${DEV_IMAGE}p2" \
  && echo_stamp "Umount loop-image" \
  && losetup -d ${DEV_IMAGE}

  set -e
}

resize_fs $1 $2
