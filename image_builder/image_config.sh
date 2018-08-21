#!/bin/bash

#
# Script for image configure
# @urpylka Artem Smirnov
# @dvornikov-aa Andrey Dvornikov
#

# Exit immidiately on non-zero result
set -e

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

  TEXT="$(date) | $1"
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

get_image() {
  # TEMPLATE: get_image $BUILD_DIR $RPI_DONWLOAD_URL $IMAGE_NAME

  local RPI_ZIP_NAME=$(basename $2)
  if [ ! -e "$1/${RPI_ZIP_NAME}" ];
  then
    echo_stamp "1. Downloading original Linux distribution"
    wget -nv -O $1/${RPI_ZIP_NAME} $2
    echo_stamp "Downloading complete" "SUCCESS"
  else
    echo_stamp "1. Linux distribution already donwloaded"
  fi
  echo_stamp "2. Unzipping Linux distribution image"
  local RPI_IMAGE_NAME=$(echo ${RPI_ZIP_NAME} | sed 's/zip/img/')
  unzip -p $1/${RPI_ZIP_NAME} ${RPI_IMAGE_NAME} > $1/$3
  echo_stamp "Unzipping complete" "SUCCESS"
}

resize_fs() {
  # TEMPLATE: resize_fs $IMAGE_PATH $SIZE

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
  && echo_stamp "Mount loop-image: $1" \
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

execute() {
  # TEMPLATE: execute $IMAGE <$EXECUTE_FILE> <...>

  echo_stamp "Mount loop-image: $1"
  local DEV_IMAGE=$(losetup -Pf $1 --show)
  sleep 0.5

  # Get temp directory to mount image
  local MOUNT_POINT=$(mktemp -d)

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
  # TEMPLATE: copy_to_chroot $IMAGE $MOVE_FILE $MOVE_TO

  echo_stamp "Mount loop-image: $1"
  local DEV_IMAGE=$(losetup -Pf $1 --show)
  sleep 0.5

  local MOUNT_POINT=$(mktemp -d)
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
  # TEMPLATE: umount_system $MOUNT_POINT $DEV_IMAGE

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

publish_image() {
  # TEMPLATE: publish_image_bash $BUILD_DIR $IMAGE_NAME $YA_SCRIPT $CONFIG_FILE $RELEASE_ID $RELEASE_BODY

  # https://developer.github.com/v3/repos/releases/

  echo_stamp "Zip image"
  if [ ! -e "$1/$2.zip" ];
  then
    cd $1 && zip $2.zip $2
    echo_stamp "Zipping complete!" "SUCCESS"
  else
    echo_stamp "Zip-archive already created"
    cd $1 && rm $2.zip && zip $2.zip $2 \
    && echo_stamp "Old archive was deleted & create new" "SUCCESS"
  fi

  echo_stamp "Upload image"
  local IMAGE_LINK=$($3 $4 $1/$2.zip) \
  && echo_stamp "Upload copmlete!" "SUCCESS"

  echo_stamp "Meashure size of zip-image"
  local IMAGE_SIZE=$(du -sh $1/$2.zip | awk '{ print $1 }') \
  && echo_stamp "Meashuring copmlete!" "SUCCESS"

  echo_stamp "Meashure hash-sum of zip-image"
  local IMAGE_HASH=$(sha256sum $1/$2.zip | awk '{ print $1 }') \
  && echo_stamp "Meashuring copmlete!" "SUCCESS"

  echo ""
  echo "\$6: $6"
  echo ""

  echo_stamp "Post message to GH"
  local NEW_RELEASE_BODY="### Download\n* [$2.zip]($IMAGE_LINK) ($IMAGE_SIZE)\nsha256: $IMAGE_HASH\n\n$6"
  local DATA="{ \"body\":\"$NEW_RELEASE_BODY\" }"

  echo ""
  echo "\$DATA: $DATA"
  echo ""

  local GH_LOGIN=$(cat $4 | jq '.github.login' -r)
  local GH_PASS=$(cat $4 | jq '.github.password' -r)
  local GH_URL=$(cat $4 | jq '.github.url' -r)
  curl -d "$DATA" -u "$GH_LOGIN:$GH_PASS" --request PATCH $GH_URL$5 \
  && echo_stamp "Post message to GH copmlete!" "SUCCESS"
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
echo "\$#: $#"
echo "\$1: $1"
echo "\$2: $2"
echo "\$3: $3"
echo "\$4: $4"
echo "\$5: $5"
echo "\$6: $6"
echo "\$7: $7"
echo "================================================================================"

case "$1" in
  get_image)
  # get_image <BUILD_DIR> <RPI_DONWLOAD_URL> <IMAGE_NAME>
    get_image $2 $3 $4;;

  resize_fs)
  # resize_fs <IMAGE_PATH> <SIZE>
    resize_fs $2 $3;;

  publish_image)
  # publish_image <BUILD_DIR> <IMAGE_NAME> <YA_SCRIPT> <CONFIG_FILE> <RELEASE_ID> <RELEASE_BODY>
    publish_image $2 $3 $4 $5 $6 "$7";;

  execute)
  # execute <IMAGE> [<EXECUTE_FILE>] [...]
    execute $2 $3 ${@:4};;

  copy_to_chroot)
  # copy_to_chroot <IMAGE> <MOVE_FILE> <MOVE_TO>
    copy_to_chroot $2 $3 $4;;

  *)
    echo "Enter one of: get_image, resize_fs, publish_image, execute";;
esac
