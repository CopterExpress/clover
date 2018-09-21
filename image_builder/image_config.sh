#! /usr/bin/env bash

#
# Script for image configure
# @urpylka Artem Smirnov
# @dvornikov-aa Andrey Dvornikov
#

# Exit immidiately on non-zero result
set -e

source echo_stamp.sh

# This script doesn't work on Ubuntu because OS`s losetup does not consist --partscan (-P).

# Idea: use `mount -o loop,offset`
# https://stefanoprenna.com/blog/2014/09/22/tutorial-how-to-mount-raw-images-img-images-on-linux/
# REPO_DIR=$(mktemp -d --suffix=.builder_repo)
# mount -t ext4 -o loop,offset=$((94208 * 512)) image/clever_qemu_test_2_20180822_163141.img "$REPO_DIR"
# mount -t vfat -o loop,offset=$((8192 * 512)) image/clever_qemu_test_2_20180822_163141.img "$REPO_DIR/boot"

execute() {
  # TEMPLATE: execute <IMAGE_PATH> <EXECUTE_FILE> <...>

  echo_stamp "Mount loop-image: $1"
  local DEV_IMAGE=$(losetup -Pf $1 --show)
  sleep 0.5

  local MOUNT_POINT=$(mktemp -d --suffix=.builder_image)
  echo_stamp "Mount dirs ${MOUNT_POINT} & ${MOUNT_POINT}/boot"
  mount "${DEV_IMAGE}p2" ${MOUNT_POINT}
  mount "${DEV_IMAGE}p1" ${MOUNT_POINT}/boot

  echo_stamp "Bind system dirs"
  echo_stamp "Mounting /proc in chroot... "
  if [ ! -d ${MOUNT_POINT}/proc ] ; then
    mkdir -p ${MOUNT_POINT}/proc \
    && echo_stamp "Created ${MOUNT_POINT}/proc" "SUCCESS"
  fi
  mount -t proc -o nosuid,noexec,nodev proc ${MOUNT_POINT}/proc \
  && echo_stamp "OK" "SUCCESS"

  echo_stamp "Mounting /sys in chroot... "
  if [ ! -d ${MOUNT_POINT}/sys ] ; then
    mkdir -p ${MOUNT_POINT}/sys \
    && echo_stamp "Created ${MOUNT_POINT}/sys" "SUCCESS"
  fi
  mount -t sysfs -o nosuid,noexec,nodev sysfs ${MOUNT_POINT}/sys \
  && echo_stamp "OK" "SUCCESS"

  echo_stamp "Mounting /dev/ and /dev/pts in chroot... " \
  && mkdir -p -m 755 ${MOUNT_POINT}/dev/pts \
  && mount -t devtmpfs -o mode=0755,nosuid devtmpfs ${MOUNT_POINT}/dev \
  && mount -t devpts -o gid=5,mode=620 devpts ${MOUNT_POINT}/dev/pts \
  && echo_stamp "OK" "SUCCESS"

  echo_stamp "Copy DNS records" \
  && cp -L /etc/resolv.conf ${MOUNT_POINT}/etc/resolv.conf

  if [[ $# > 1 ]]; then
    echo_stamp "Entering to chroot"
    local script_name=$(basename $2)
    # TODO: maybe copy to tmp-dir
    local script_path_root="${MOUNT_POINT}/root/${script_name}"
    # Copy script into chroot fs
    # TODO: Find more suitable location for temporary script storage
    cp "$2" "${script_path_root}"
    # Its important to save arguments (direct ${@:4} causes problems)
    script_args="${@:3}"
    # Run script in chroot with additional arguments
    chroot ${MOUNT_POINT} /bin/sh -c "/root/${script_name} ${script_args}"
    # Removing script from chroot fs
    rm "${script_path_root}"
  else
    # https://wiki.archlinux.org/index.php/Change_root_(%D0%A0%D1%83%D1%81%D1%81%D0%BA%D0%B8%D0%B9)
    # http://www.unix-lab.org/posts/chroot/
    # https://habrahabr.ru/post/141012/
    # https://losst.ru/vosstanovlenie-grub2
    # http://unixteam.ru/content/virtualizaciya-ili-zapuskaem-prilozhenie-v-chroot-okruzhenii-razmyshleniya
    # http://help.ubuntu.ru/wiki/%D0%B2%D0%BE%D1%81%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BB%D0%B5%D0%BD%D0%B8%D0%B5_grub
    echo_stamp "Entering to chroot" \
    && chroot ${MOUNT_POINT} /bin/bash
  fi

  umount_system ${MOUNT_POINT} ${DEV_IMAGE}
}

copy_to_chroot() {
  # TEMPLATE: copy_to_chroot <IMAGE_PATH> <MOVE_FILE> <MOVE_TO>

  echo_stamp "Mount loop-image: $1"
  local DEV_IMAGE=$(losetup -Pf $1 --show)
  sleep 0.5

  local MOUNT_POINT=$(mktemp -d --suffix=.builder_image)
  echo_stamp "Mount dirs ${MOUNT_POINT} & ${MOUNT_POINT}/boot"
  mount "${DEV_IMAGE}p2" ${MOUNT_POINT}
  mount "${DEV_IMAGE}p1" ${MOUNT_POINT}/boot

  dir_name=$(dirname "${MOUNT_POINT}$3 /")
  if [ ! -d ${dir_name} ] ; then
    mkdir -p ${dir_name} \
    && echo_stamp "Created ${dir_name}" "SUCCESS"
  fi

  cp -r "$2" "${MOUNT_POINT}$3"
  umount_system ${MOUNT_POINT} ${DEV_IMAGE}
}

umount_system() {
  # TEMPLATE: umount_system <MOUNT_POINT> <DEV_IMAGE>

  echo_stamp "Umount recursive dirs: $1"
  # There is a risk that umount will fail
  set +e
  # Successfull unmount flag (false at thismoment)
  umount_ok=false
  # Repeat 5 times
  for i in {1..5}
  do
    # Unmount chroot rootfs and boot partition
    umount -fR $1
    # If no problems detected
    if [[ $? == 0 ]]
    then
    echo_stamp "Successfull unmount" "SUCCESS"
    # Set flag
    umount_ok=true
    # Exit loop
    break
    fi
    # Unmount has failed
    echo_stamp "Unmount failed" "ERROR"
    # Wait for some time
    sleep 2
  done
  set -e
  # Jenkins job will fail if this condition is not true
  [[ "$umount_ok" == true ]]
  echo_stamp "Umount loop-image"
  losetup -d $2
}

if [ $(whoami) != "root" ];
then echo "" \
  && echo "********************************************************************" \
  && echo "******************** This should be run as root ********************" \
  && echo "********************************************************************" \
  && echo "" \
  && exit 1
fi

echo "================================================================================"
for ((i=1; i<=$#; i++)); do echo "\$$i: ${!i}"; done
echo "================================================================================"

case "$1" in
  execute)
  # execute <IMAGE_PATH> [<EXECUTE_FILE>] [...]
    execute $2 $3 ${@:4};;

  copy_to_chroot)
  # copy_to_chroot <IMAGE_PATH> <MOVE_FILE> <MOVE_TO>
    copy_to_chroot $2 $3 $4;;

  *)
    echo "Enter one of: get_image, resize_fs, publish_image, execute";;
esac
