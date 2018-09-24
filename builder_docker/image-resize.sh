#! /usr/bin/env bash

#
# Script for upsize the image & for shrink free space on the image
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Artem Smirnov <urpylka@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -e # Exit immidiately on non-zero result

echo_bold() {
  # TEMPLATE: echo_bold <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

  TEXT="$1"
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

  echo_bold "Truncate image" \
  && truncate -s$2 $1 \
  && echo_bold "Mount loop-image: $1" \
  && local DEV_IMAGE=$(losetup -Pf $1 --show) \
  && sleep 0.5 \
  && echo ", +" | sfdisk -N 2 ${DEV_IMAGE} \
  && sleep 0.5 \
  && losetup -d ${DEV_IMAGE} \
  && sleep 0.5 \
  && local DEV_IMAGE=$(losetup -Pf $1 --show) \
  && sleep 0.5 \
  && echo_bold "Check & repair filesystem after expand partition" \
  && e2fsck -fvy "${DEV_IMAGE}p2" \
  && echo_bold "Expand filesystem" \
  && resize2fs "${DEV_IMAGE}p2" \
  && echo_bold "Umount loop-image" \
  && losetup -d ${DEV_IMAGE}

  set -e
}

shrink_free_space() {
  if [[ -z $1 ]]; then
    echo "================================================================================"
    echo_bold "Automatic Image file resizer"
    echo_bold "Description: This script shrink your image to 10MiB free space"
    echo_bold "if you didn't set FREE_SPACE in MiB (see usage below)."
    echo_bold "Authors: Artem Smirnov @urpylka, SirLagz"
    echo
    echo_bold "Usage: ./autosizer.sh <IMAGE_PATH> [<FREE_SPACE>]"
    echo
    echo_bold "Requirements: parted, losetup, e2fsck, resize2fs, bc, truncate"
    echo "================================================================================"
    exit 1
  fi

  echo "================================================================================"
  # Default add 10MiB free space to image, if $2 doesn't set
  FREE_SPACE=${2:-10}

  strImgFile=$1
  echo_bold "Path to image: $strImgFile"
  echo "================================================================================"

  if [[ ! -e $strImgFile ]]; then
    echo_bold "Error: File doesn't exist"
    echo
    exit 1
  fi

  echo "================================================================================"
  partinfo=`parted -m $strImgFile unit B print`
  echo_bold "Partition information:\n$partinfo"
  echo "================================================================================"

  partnumber=`echo "$partinfo" | grep ext4 | awk -F: '{ print $1 }'`
  echo_bold "Partition number: $partnumber"
  echo "================================================================================"

  partstart=`echo "$partinfo" | grep ext4 | awk -F: '{ print substr($2,0,length($2)-1) }'`
  echo_bold "Partition start: $partstart (bytes)"
  echo "================================================================================"

  loopback=`losetup -f --show -o $partstart $strImgFile`
  echo_bold "Loopback device: $loopback"
  echo "================================================================================"

  set +e
  e2fsck -fvy $loopback
  set -e

  echo "================================================================================"
  minsize=`resize2fs -P $loopback | awk -F': ' '{ print $2 }'`
  #minsize=`resize2fs -P $loopback 2> /dev/null | awk -F': ' '{ print $2 }'`
  echo_bold "Minsize: $minsize (4KiB)"
  echo "================================================================================"

  FREE_SPACE=$(($FREE_SPACE*1024*1024/4096))

  minsize=`echo "$minsize+$FREE_SPACE" | bc`
  echo_bold "Minsize + $FREE_SPACE (4KiB): $minsize (4KiB)"
  echo "================================================================================"

  resize2fs -p $loopback $minsize
  sleep 1
  losetup -d $loopback

  echo "================================================================================"
  partnewsize=`echo "$minsize * 4096" | bc`
  echo_bold "New size of part: $minsize (4KiB) = $partnewsize (bytes)"
  echo "================================================================================"

  newpartend=`echo "$partstart + $partnewsize" | bc`
  echo_bold "New end of part (Part start + part new size):"
  echo_bold "$partstart (bytes) + $partnewsize (bytes) = $newpartend (bytes)"
  echo "================================================================================"

  part1=`parted $strImgFile rm 2`
  echo "================================================================================"
  part2=`parted $strImgFile unit B mkpart primary $partstart $newpartend`

  echo "================================================================================"
  endresult=`parted -m $strImgFile unit B print free | tail -1 | awk -F: '{ print substr($2,0,length($2)-1) }'`
  echo_bold "Size of result image: $endresult (bytes)"
  echo "================================================================================"

  truncate -s $endresult $strImgFile

  echo "================================================================================"
  partinfo=`parted -m $strImgFile unit B print`
  echo_bold "Partition information:\n$partinfo"
  echo "================================================================================"

  # TODO check if image needs to change PARTUUID
  #sed -i 's/root=[^ ]*/root=\/dev\/mmcblk0p2/' /boot/cmdline.txt
  #sed -i 's/.*  \/boot           vfat    defaults          0       2$/\/dev\/mmcblk0p1  \/boot           vfat    defaults          0       2/' /etc/fstab
  #sed -i 's/.*  \/               ext4    defaults,noatime  0       1$/\/dev\/mmcblk0p2  \/               ext4    defaults,noatime  0       1/' /etc/fstab
}

if [ $(whoami) != "root" ]; then
  echo ""
  echo "********************************************************************"
  echo "******************** This should be run as root ********************"
  echo "********************************************************************"
  echo ""
  exit 1
fi

if [[ $# > 0 ]]; then
  echo "================================================================================"
  for ((i=1; i<=$#; i++)); do echo "\$$i: ${!i}"; done
  echo "================================================================================"

  [[ -f $1 ]] || (echo_bold "$1 does not exist" "ERROR"; exit 1)

  if [[ ! -z $2 ]]; then
    case "$2" in
      min)
        shrink_free_space $1 $3;;
      max)
        resize_fs $1 $3;;
      *)
        echo "Template: image-resize.sh <IMAGE> [ min <FREE_SPACE> | max <FREE_SPACE> ]";;
    esac
  else shrink_free_space $1; fi
fi
