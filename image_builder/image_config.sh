#!/bin/bash

# Exit immidiately on non-zero result
set -e

#
# Script for image configure
# @urpylka Artem Smirnov
# @dvornikov-aa Andrey Dvornikov
#

get_image() {

# STATIC FUNCTION
# TEMPLATE: get_image $BUILD_DIR $RPI_DONWLOAD_URL $IMAGE_NAME

  local RPI_ZIP_NAME=$(basename $2)
  if [ ! -e "$1/${RPI_ZIP_NAME}" ];
  then
    echo "$(date) | 1. Downloading original Linux distribution"
    wget -nv -O $1/${RPI_ZIP_NAME} $2
    echo "$(date) | Downloading complete"
  else
    echo "$(date) | 1. Linux distribution already donwloaded"
  fi
  echo "$(date) | 2. Unzipping Linux distribution image"
  local RPI_IMAGE_NAME=$(echo ${RPI_ZIP_NAME} | sed 's/zip/img/')
  unzip -p $1/${RPI_ZIP_NAME} ${RPI_IMAGE_NAME} > $1/$3
  echo "$(date) | Unzipping complete"
}

resize_fs() {

  # STATIC FUNCTION
  # TEMPLATE: resize_fs $IMAGE_PATH $SIZE

  # Partitions numbers
  local BOOT_PARTITION=1
  local ROOT_PARTITION=2

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

  echo -e "\033[0;31m\033[1mTruncate image\033[0m\033[0m" \
    && truncate -s$2 $1 \
    && echo "Mount loop-image: $1" \
    && local DEV_IMAGE=$(losetup -Pf $1 --show) \
    && sleep 0.5 \
    && echo -e "\033[0;31m\033[1mMount loop-image: $1\033[0m\033[0m" \
    && echo ", +" | sfdisk -N ${ROOT_PARTITION} ${DEV_IMAGE} \
    && sleep 0.5 \
    && losetup -d ${DEV_IMAGE} \
    && sleep 0.5 \
    && local DEV_IMAGE=$(losetup -Pf $1 --show) \
    && sleep 0.5 \
    && echo -e "\033[0;31m\033[1mCheck & repair filesystem after expand partition\033[0m\033[0m" \
    && e2fsck -fvy "${DEV_IMAGE}p${ROOT_PARTITION}" \
    && echo -e "\033[0;31m\033[1mExpand filesystem\033[0m\033[0m" \
    && resize2fs "${DEV_IMAGE}p${ROOT_PARTITION}" \
    && echo -e "\033[0;31m\033[1mUmount loop-image\033[0m\033[0m" \
    && losetup -d ${DEV_IMAGE}

  set -e
}

execute() {

  # STATIC FUNCTION
  # TEMPLATE: execute $IMAGE <$EXECUTE_FILE> <...>

  # Partitions numbers
  local BOOT_PARTITION=1
  local ROOT_PARTITION=2

  echo -e "\033[0;31m\033[1mMount loop-image: $1\033[0m\033[0m"
  local DEV_IMAGE=$(losetup -Pf $1 --show)
  sleep 0.5

  # Get temp directory to mount image
  local MOUNT_POINT=$(mktemp -d)

  echo -e "\033[0;31m\033[1mMount dirs ${MOUNT_POINT} & ${MOUNT_POINT}/boot\033[0m\033[0m"
  mount "${DEV_IMAGE}p${ROOT_PARTITION}" ${MOUNT_POINT}
  mount "${DEV_IMAGE}p${BOOT_PARTITION}" ${MOUNT_POINT}/boot

  echo -e "\033[0;31m\033[1mBind system dirs\033[0m\033[0m"
  echo "Mounting /proc in chroot... "
  if [ ! -d ${MOUNT_POINT}/proc ] ; then
    mkdir -p ${MOUNT_POINT}/proc
    echo "Created ${MOUNT_POINT}/proc"
  fi
  mount -t proc -o nosuid,noexec,nodev proc ${MOUNT_POINT}/proc \
    && echo "OK"

  echo "Mounting /sys in chroot... "
  if [ ! -d ${MOUNT_POINT}/sys ] ; then
    mkdir -p ${MOUNT_POINT}/sys
    echo "Created ${MOUNT_POINT}/sys"
  fi
  mount -t sysfs -o nosuid,noexec,nodev sysfs ${MOUNT_POINT}/sys \
    && echo "OK"

  echo "Mounting /dev/ and /dev/pts in chroot... " \
    && mkdir -p -m 755 ${MOUNT_POINT}/dev/pts \
    && mount -t devtmpfs -o mode=0755,nosuid devtmpfs ${MOUNT_POINT}/dev \
    && mount -t devpts -o gid=5,mode=620 devpts ${MOUNT_POINT}/dev/pts \
    && echo "OK"

  echo -e "\033[0;31m\033[1mCopy DNS records\033[0m\033[0m" \
    && cp -L /etc/resolv.conf ${MOUNT_POINT}/etc/resolv.conf

  if [[ $# > 1 ]]; then
    echo -e "\033[0;31m\033[1m$(date) | Enter chroot\033[0m\033[0m"
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
    echo -e "\033[0;31m\033[1mEnter chroot\033[0m\033[0m" \
      && chroot ${MOUNT_POINT} /bin/bash
  fi

  umount_system ${MOUNT_POINT} ${DEV_IMAGE}
}

copy_to_chroot() {

  # STATIC FUNCTION
  # TEMPLATE: copy_to_chroot $IMAGE $MOVE_FILE $MOVE_TO

  # Partitions numbers
  local BOOT_PARTITION=1
  local ROOT_PARTITION=2

  echo -e "\033[0;31m\033[1mMount loop-image: $1\033[0m\033[0m"
  local DEV_IMAGE=$(losetup -Pf $1 --show)
  sleep 0.5

  # Get temp directory to mount image
  local MOUNT_POINT=$(mktemp -d)

  echo -e "\033[0;31m\033[1mMount dirs ${MOUNT_POINT} & ${MOUNT_POINT}/boot\033[0m\033[0m"
  mount "${DEV_IMAGE}p${ROOT_PARTITION}" ${MOUNT_POINT}
  mount "${DEV_IMAGE}p${BOOT_PARTITION}" ${MOUNT_POINT}/boot

  echo -e "\033[0;31m\033[1m$(date) | Enter chroot\033[0m\033[0m"
  file_name=$(basename $2)
  file_path_root="${MOUNT_POINT}$3/${file_name}"
  # Copy script into chroot fs
  # TODO: Find more suitable location for temporary script storage
  if [ ! -d ${file_path_root} ] ; then
    mkdir -p ${file_path_root} \
      && echo "Created ${file_path_root}"
  fi
  cp "$2" "${file_path_root}"

  umount_system ${MOUNT_POINT} ${DEV_IMAGE}
}

umount_system() {

  # STATIC FUNCTION
  # TEMPLATE: umount_system $MOUNT_POINT $DEV_IMAGE

  echo -e "\033[0;31m\033[1m$(date) | Umount recursive dirs: $1\033[0m\033[0m"
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
    echo -e "\033[0;31m\033[1m$(date) | Successfull unmount\033[0m\033[0m"
    # Set flag
    umount_ok=true
    # Exit loop
    break
    fi
    # Unmount has failed
    echo -e "\033[0;31m\033[1m$(date) | Unmount failed\033[0m\033[0m"
    # Wait for some time
    sleep 2
  done
  set -e
  # Jenkins job will fail if this condition is not true
  [[ "$umount_ok" == true ]]
  echo -e "\033[0;31m\033[1m$(date) | Umount loop-image\033[0m\033[0m"
  #losetup -d $DEV_IMAGE
  losetup -d $2
}

publish_image() {

# STATIC FUNCTION
# TEMPLATE: publish_image_bash $BUILD_DIR $IMAGE_NAME $YA_SCRIPT $CONFIG_FILE $RELEASE_ID $RELEASE_BODY

# https://developer.github.com/v3/repos/releases/
#RELEASE_BODY="### Changelog\n* Add /boot/cmdline.txt net.ifnames=0 https://www.freedesktop.org/wiki/Software/systemd/PredictableNetworkInterfaceNames/\n* Updated cophelper\n* Installed copstat"

  echo -e "\033[0;31m\033[1m$(date) | Zip image\033[0m\033[0m"
  if [ ! -e "$1/$2.zip" ];
  then
    cd $1 && zip $2.zip $2
    echo -e "\033[0;31m\033[1m$(date) | Zipping complete!\033[0m\033[0m"
  else
    echo -e "\033[0;31m\033[1m$(date) | Zip-archive already created\033[0m\033[0m"
    cd $1 && rm $2.zip && zip $2.zip $2
    echo -e "\033[0;31m\033[1m$(date) | Old archive was deleted & create new\033[0m\033[0m"
  fi

  echo -e "\033[0;31m\033[1m$(date) | Upload image\033[0m\033[0m"
  local IMAGE_LINK=$($3 $4 $1/$2.zip)
  echo -e "\033[0;31m\033[1m$(date) | Upload copmlete!\033[0m\033[0m"

  echo -e "\033[0;31m\033[1m$(date) | Meashure size of zip-image\033[0m\033[0m"
  local IMAGE_SIZE=$(du -sh $1/$2.zip | awk '{ print $1 }')
  echo -e "\033[0;31m\033[1m$(date) | Meashuring copmlete!\033[0m\033[0m"

  echo -e "\033[0;31m\033[1m$(date) | Meashure hash-sum of zip-image\033[0m\033[0m"
  local IMAGE_HASH=$(sha256sum $1/$2.zip | awk '{ print $1 }')
  echo -e "\033[0;31m\033[1m$(date) | Meashuring copmlete!\033[0m\033[0m"

  echo ""
  echo "\$6: $6"
  echo ""

  echo -e "\033[0;31m\033[1m$(date) | Post message to GH\033[0m\033[0m"
  local NEW_RELEASE_BODY="### Download\n* [$2.zip]($IMAGE_LINK) ($IMAGE_SIZE)\nsha256: $IMAGE_HASH\n\n$6"
  local DATA="{ \"body\":\"$NEW_RELEASE_BODY\" }"

  echo ""
  echo "\$DATA: $DATA"
  echo ""

  local GH_LOGIN=$(cat $4 | jq '.github.login' -r)
  local GH_PASS=$(cat $4 | jq '.github.password' -r)
  local GH_URL=$(cat $4 | jq '.github.url' -r)
  curl -d "$DATA" -u "$GH_LOGIN:$GH_PASS" --request PATCH $GH_URL$5
  echo -e "\033[0;31m\033[1m$(date) | Post message to GH copmlete!\033[0m\033[0m"
}

if [ $(whoami) != "root" ];
then echo "" \
  && echo "********************************************************************" \
  && echo "******************** This should be run as root ********************" \
  && echo "********************************************************************" \
  && echo "" \
  && exit 1
fi

echo "\$#: $#"
echo "\$1: $1"
echo "\$2: $2"
echo "\$3: $3"
echo "\$4: $4"
echo "\$5: $5"
echo "\$6: $6"
echo "\$7: $7"

case "$1" in
  get_image)
  # get_image $BUILD_DIR $RPI_DONWLOAD_URL $IMAGE_NAME
    get_image $2 $3 $4;;

  resize_fs)
  # resize_fs $IMAGE_PATH $SIZE
    resize_fs $2 $3;;

  publish_image)
  # publish_image $BUILD_DIR $IMAGE_NAME $YA_SCRIPT $CONFIG_FILE $RELEASE_ID $RELEASE_BODY
    publish_image $2 $3 $4 $5 $6 "$7";;

  execute)
  # execute $IMAGE $EXECUTE_FILE ...
    execute $2 $3 ${@:4};;

  copy_to_chroot)
  # copy_to_chroot $IMAGE $MOVE_FILE $MOVE_TO
    copy_to_chroot $2 $3 $4;;

  *)
    echo "Enter one of: get_image, resize_fs, publish_image, execute";;
esac
