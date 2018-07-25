#!/bin/bash

set -e

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
  echo -e "\033[0;31m\033[1mAutomatic Image file resizer\033[0m\033[0m"
  echo -e "\033[0;31m\033[1mDescription:\033[0m\033[0m This script shrink your image to 10MiB free space"
  echo -e "if you didn't set FREE_SPACE in MiB (see usage below)."
  echo -e "\033[0;31m\033[1mAuthors:\033[0m\033[0m Artem Smirnov @urpylka, SirLagz"
  echo
  echo -e "\033[0;31m\033[1mUsage:\033[0m\033[0m ./autosizer.sh PATH_TO_IMAGE FREE_SPACE"
  echo
  echo -e "\033[0;31m\033[1mRequirements:\033[0m\033[0m parted, losetup, e2fsck, resize2fs, bc, truncate"
  echo "================================================================================"
  exit 0
fi

echo "================================================================================"
strImgFile=$1
echo -e "\033[0;31m\033[1mPath to image: $strImgFile\033[0m\033[0m"
echo "================================================================================"

if [[ ! -e $strImgFile ]]; then
  echo -e "\033[0;31m\033[1mError: File doesn't exist\033[0m\033[0m"
  echo
  exit 1
fi

echo "================================================================================"
partinfo=`parted -m $strImgFile unit B print`
echo -e "\033[0;31m\033[1mPartition information:\033[0m\033[0m\n$partinfo"
echo "================================================================================"

partnumber=`echo "$partinfo" | grep ext4 | awk -F: '{ print $1 }'`
echo -e "\033[0;31m\033[1mPartition number: $partnumber\033[0m\033[0m"
echo "================================================================================"

partstart=`echo "$partinfo" | grep ext4 | awk -F: '{ print substr($2,0,length($2)-1) }'`
echo -e "\033[0;31m\033[1mPartition start: $partstart (bytes)\033[0m\033[0m"
echo "================================================================================"

loopback=`losetup -f --show -o $partstart $strImgFile`
echo -e "\033[0;31m\033[1mLoopback device: $loopback\033[0m\033[0m"
echo "================================================================================"

set +e
e2fsck -fvy $loopback
set -e

echo "================================================================================"
minsize=`resize2fs -P $loopback | awk -F': ' '{ print $2 }'`
#minsize=`resize2fs -P $loopback 2> /dev/null | awk -F': ' '{ print $2 }'`
echo -e "\033[0;31m\033[1mMinsize: $minsize (4KiB)\033[0m\033[0m"
echo "================================================================================"

# Default add 10MiB free space to image, if $2 doesn't set
FREE_SPACE=${2:-10}

FREE_SPACE=$(($FREE_SPACE*1024*1024/4096))

minsize=`echo "$minsize+$FREE_SPACE" | bc`
echo -e "\033[0;31m\033[1mMinsize + $FREE_SPACE (4KiB): $minsize (4KiB)\033[0m\033[0m"
echo "================================================================================"

resize2fs -p $loopback $minsize
sleep 1
losetup -d $loopback

echo "================================================================================"
partnewsize=`echo "$minsize * 4096" | bc`
echo -e "\033[0;31m\033[1mNew size of part: $minsize (4KiB) = $partnewsize (bytes)\033[0m\033[0m"
echo "================================================================================"

newpartend=`echo "$partstart + $partnewsize" | bc`
echo -e "\033[0;31m\033[1mNew end of part (Part start + part new size):\033[0m\033[0m"
echo -e "\033[0;31m\033[1m$partstart (bytes) + $partnewsize (bytes) = $newpartend (bytes)\033[0m\033[0m"
echo "================================================================================"

part1=`parted $strImgFile rm 2`
echo "================================================================================"
part2=`parted $strImgFile unit B mkpart primary $partstart $newpartend`

echo "================================================================================"
endresult=`parted -m $strImgFile unit B print free | tail -1 | awk -F: '{ print substr($2,0,length($2)-1) }'`
echo -e "\033[0;31m\033[1mSize of result image: $endresult (bytes)\033[0m\033[0m"
echo "================================================================================"

truncate -s $endresult $strImgFile

echo "================================================================================"
partinfo=`parted -m $strImgFile unit B print`
echo -e "\033[0;31m\033[1mPartition information:\033[0m\033[0m\n$partinfo"
echo "================================================================================"
