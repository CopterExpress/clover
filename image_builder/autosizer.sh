#!/bin/bash

set -e

echo_bold() {

  # STATIC FUNCTION
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

if [ $(whoami) != "root" ]; then
  echo \
  && echo "********************************************************************" \
  && echo "******************** This should be run as root ********************" \
  && echo "********************************************************************" \
  && echo \
  && exit 1
fi

if [[ -z $1 ]]; then
  echo "================================================================================"
  echo_bold "Automatic Image file resizer"
  echo_bold "Description: This script shrink your image to 10MiB free space"
  echo -e "if you didn't set FREE_SPACE in MiB (see usage below)."
  echo_bold "Authors: Artem Smirnov @urpylka, SirLagz"
  echo
  echo_bold "Usage: ./autosizer.sh PATH_TO_IMAGE FREE_SPACE"
  echo
  echo_bold "Requirements: parted, losetup, e2fsck, resize2fs, bc, truncate"
  echo "================================================================================"
  exit 0
fi

echo "================================================================================"
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

# Default add 10MiB free space to image, if $2 doesn't set
FREE_SPACE=${2:-10}

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
