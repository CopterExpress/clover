#!/bin/bash

# Exit immidiately on non-zero result
set -e

#
# Script for image configure
# @smirart Smirnov Artem
# @Andrey Dvornikov
#

get_image() {

# STATIC
# TEMPLATE: get_image $BUILD_DIRECTORY $RPI_ZIP_NAME $RPI_DONWLOAD_URL $RPI_IMAGE_NAME $IMAGE_NAME

  echo 'Download original Linux distribution'
  echo "$(date) | 1. Downloading Linux distribution"
  if [ ! -e "$1/$2" ];
  then wget -nv -O $1/$2 $3
  fi
  echo "$(date) | Downloading complete"
  echo 'Unzip image'
  echo "$(date) | 2. Unzipping Linux distribution image"
  #if [ ! -e "$1/$4" ];
  #then unzip -uo $1/$2 -d $1
  unzip -p $1/$2 $4 > $1/$5
  #fi
  #echo "$(date) | Unziping complete"
  #echo 'Duplicate image'
  #cp -f $1/$4 $1/$5
}

resize_fs() {

  # STATIC
  # TEMPLATE: resize_fs $SIZE $BUILD_DIRECTORY $IMAGE_NAME $ROOT_PARTITION

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
  # ", +" : расширяет раздел до размеров образа
  # -N 2  : выбирает раздел 2 для работы

  echo "\033[0;31m\033[1mTruncate image\033[0m\033[0m" \
    && truncate -s$1 $2/$3 \
    && echo "Mount loop-image: $2/$3" \
    && local DEV_IMAGE=$(losetup -Pf $2/$3 --show) \
    && sleep 0.5 \
    && echo "\033[0;31m\033[1mMount loop-image: $1\033[0m\033[0m" \
    && echo ", +" | sfdisk -N 2 $DEV_IMAGE \
    && sleep 0.5 \
    # There is a risk that sfdisk will ask for a disk remount to update partition table
    # TODO: Check sfdisk exit code
    && losetup -d $DEV_IMAGE \
    && sleep 0.5 \
    && local DEV_IMAGE=$(losetup -Pf $2/$3 --show) \
    && sleep 0.5 \
    && echo "\033[0;31m\033[1mCheck & repair filesystem after expand partition\033[0m\033[0m" \
    && e2fsck -fvy "${DEV_IMAGE}p$ROOT_PARTITION" \
    && echo "\033[0;31m\033[1mExpand filesystem\033[0m\033[0m" \
    && resize2fs "${DEV_IMAGE}p$ROOT_PARTITION" \
    && echo "\033[0;31m\033[1mUmount loop-image\033[0m\033[0m" \
    && losetup -d $DEV_IMAGE

  set -e
}

publish_image() {

# STATIC
# TEMPLATE: publish_image $BUILD_DIRECTORY $IMAGE_NAME $WORKSPACE $CONFIG_FILE $RELEASE_ID $RELEASE_BODY

# https://developer.github.com/v3/repos/releases/
#RELEASE_BODY="### Changelog\n* Add /boot/cmdline.txt net.ifnames=0 https://www.freedesktop.org/wiki/Software/systemd/PredictableNetworkInterfaceNames/\n* Updated cophelper\n* Installed copstat"

  echo 'Zip image' \
    && zip $1/$2.zip $1/$2 \
    && echo 'Upload image' \
    && local IMAGE_LINK=$($3/image/yadisk.py $1/$4 $1/$2.zip) \
    && local IMAGE_SIZE=$(du -sh $1/$2.zip | awk '{ print $1 }') \
    && echo "Make downloads in GH-release" \
    && $3/image/git_release.py $1/$4 $5 $6 $2 $IMAGE_LINK $IMAGE_SIZE
#    echo "Fake publish"
}

publish_image2() {

# STATIC
# TEMPLATE: publish_image $BUILD_DIRECTORY $IMAGE_NAME $WORKSPACE $CONFIG_FILE $RELEASE_ID $RELEASE_BODY

# https://developer.github.com/v3/repos/releases/
#RELEASE_BODY="### Changelog\n* Add /boot/cmdline.txt net.ifnames=0 https://www.freedesktop.org/wiki/Software/systemd/PredictableNetworkInterfaceNames/\n* Updated cophelper\n* Installed copstat"

  echo 'Zip image' \
    && zip $1/$2.zip $1/$2 \
    && echo 'Upload image' \
    && local IMAGE_LINK=$($3/image/yadisk.py $1/$4 $1/$2.zip) \
    && local IMAGE_SIZE=$(du -sh $1/$2.zip | awk '{ print $1 }') \
    && local NEW_RELEASE_BODY="### Download\n* [$2.zip]($IMAGE_LINK) ($IMAGE_SIZE)\n\n$6" \
    && local DATA="{ \"body\":\"$NEW_RELEASE_BODY\" }" \
    && curl -d "$(echo $DATA)" -u "LOGIN:PASS" --request PATCH https://api.github.com/repos/ONWER/REPO/releases/$5
}

burn_image() {

# STATIC
# TEMPLATE: burn_image $IMAGE_PATH $MICROSD_DEV

  echo "\033[0;31m\033[1mBurn image\033[0m\033[0m" \
    && dd if=$1 of=$2 \
    && echo "\033[0;31m\033[1mBurn image finished!\033[0m\033[0m"
}

burn_and_reboot() {

# STATIC
# TEMPLATE: burn_and_reboot $IMAGE_PATH $MICROSD_DEV

  burn_image $1 $2 \
    && reboot
}

mount_system() {

  # STATIC
  # TEMPLATE: mount_system $IMAGE $PREFIX_PATH $ROOT_PARTITION $BOOT_PARTITION

  # https://www.stableit.ru/2011/05/losetup.html
  # -f     : losetup выбирает незанятое имя устройства, например /dev/loop2
  # -P     : losetup монтирует разделы в образе как отдельные подразделы,
  #          например /dev/loop0p1 и /dev/loop0p2
  # --show : печатает имя устройства, например /dev/loop4

  echo "\033[0;31m\033[1mMount loop-image: $1\033[0m\033[0m"
  DEV_IMAGE=$(losetup -Pf $1 --show)
  sleep 0.5

  echo "\033[0;31m\033[1mMount dirs $2 & $2/boot\033[0m\033[0m"
  #mount $3 $2
  #mount $4 $2/boot
  mount "{$DEV_IMAGE}p$ROOT_PARTITION" $2
  mount "{$DEV_IMAGE}p$BOOT_PARTITION" $2/boot

  echo "\033[0;31m\033[1mBind system dirs\033[0m\033[0m"
  # https://github.com/debian-pi/raspbian-ua-netinst/issues/314
  echo "Mounting /proc in chroot... "
  if [ ! -d $2/proc ] ; then
      mkdir -p $2/proc
      echo "Created $2/proc"
  fi
  mount -t proc -o nosuid,noexec,nodev proc $2/proc
  echo "OK"
  
  echo "Mounting /sys in chroot... "
  if [ ! -d $2/sys ] ; then
      mkdir -p $2/sys
      echo "Created $2/sys"
  fi
  mount -t sysfs -o nosuid,noexec,nodev sysfs $2/sys
  echo "OK"
  
  echo "Mounting /dev/ and /dev/pts in chroot... "
  mkdir -p -m 755 $2/dev/pts
  mount -t devtmpfs -o mode=0755,nosuid devtmpfs $2/dev
  mount -t devpts -o gid=5,mode=620 devpts $2/dev/pts
  # mount -t devpts none "$2/dev/pts" -o ptmxmode=0666,newinstance
  # ln -fs "pts/ptmx" "$2/dev/ptmx"
  echo "OK"


  # mount -o bind /dev $2/dev
  # mount -t proc proc $2/proc
  # mount -t devpts devpts $2/dev/pts

  # mount -t proc proc $2/proc
  # mount -t sysfs sys $2/sys
  # mount --bind /dev $2/dev

  echo "\033[0;31m\033[1mCopy DNS records\033[0m\033[0m"
  cp -L /etc/resolv.conf $2/etc/resolv.conf

  # https://wiki.archlinux.org/index.php/Change_root_(%D0%A0%D1%83%D1%81%D1%81%D0%BA%D0%B8%D0%B9)
  # http://www.unix-lab.org/posts/chroot/
  # https://habrahabr.ru/post/141012/
  # https://losst.ru/vosstanovlenie-grub2
  # http://unixteam.ru/content/virtualizaciya-ili-zapuskaem-prilozhenie-v-chroot-okruzhenii-razmyshleniya
  # http://help.ubuntu.ru/wiki/%D0%B2%D0%BE%D1%81%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BB%D0%B5%D0%BD%D0%B8%D0%B5_grub
  echo "\033[0;31m\033[1mEnter chroot\033[0m\033[0m"
  chroot $2 /bin/bash
}

mount_system2() {

  # STATIC
  # TEMPLATE: mount_system2 $IMAGE $PREFIX_PATH $ROOT_PARTITION $BOOT_PARTITION $EXECUTE_FILE ...

  echo "\033[0;31m\033[1mMount loop-image: $1\033[0m\033[0m"
  DEV_IMAGE=$(losetup -Pf $1 --show)
  sleep 0.5

  echo "\033[0;31m\033[1mMount dirs $2 & $2/boot\033[0m\033[0m"
  mount "${DEV_IMAGE}p$ROOT_PARTITION" $2
  mount "${DEV_IMAGE}p$BOOT_PARTITION" $2/boot

  echo "\033[0;31m\033[1mBind system dirs\033[0m\033[0m"
  echo "Mounting /proc in chroot... "
  if [ ! -d $2/proc ] ; then
      mkdir -p $2/proc
      echo "Created $2/proc"
  fi
  mount -t proc -o nosuid,noexec,nodev proc $2/proc
  echo "OK"
  
  echo "Mounting /sys in chroot... "
  if [ ! -d $2/sys ] ; then
      mkdir -p $2/sys
      echo "Created $2/sys"
  fi
  mount -t sysfs -o nosuid,noexec,nodev sysfs $2/sys
  echo "OK"
  
  echo "Mounting /dev/ and /dev/pts in chroot... "
  mkdir -p -m 755 $2/dev/pts
  mount -t devtmpfs -o mode=0755,nosuid devtmpfs $2/dev
  mount -t devpts -o gid=5,mode=620 devpts $2/dev/pts
  echo "OK"

  echo "\033[0;31m\033[1mCopy DNS records\033[0m\033[0m"
  cp -L /etc/resolv.conf $2/etc/resolv.conf

  echo "\033[0;31m\033[1m$(date) | Enter chroot\033[0m\033[0m"
  script_name=$(basename $5)
  script_path_root="$2/root/$script_name"
  # Copy script into chroot fs
  # TODO: Find more suitable location for temporary script storage
  cp "$5" "$script_path_root"
  # It's important to save arguments (direct ${@:6} causes problems)
  script_args="${@:6}"
  # Run script in chroot with additional arguments
  chroot $2 /bin/sh -c "/root/$script_name $script_args"
  # Removing script from chroot fs
  rm "$script_path_root"
}

umount_system() {

  # STATIC
  # TEMPLATE: umount_system $PREFIX_PATH

  echo "\033[0;31m\033[1m$(date) | Umount recursive dirs: $1\033[0m\033[0m"
  umount -fR $1
  echo "\033[0;31m\033[1m$(date) | Umount loop-image\033[0m\033[0m"
  losetup -d $DEV_IMAGE
}

umount_system2() {

  # STATIC
  # TEMPLATE: umount_system $PREFIX_PATH
  
  echo "\033[0;31m\033[1m$(date) | Umount recursive dirs: $1\033[0m\033[0m"
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
	echo "\033[0;31m\033[1m$(date) | Successfull unmount\033[0m\033[0m"
	# Set flag
	umount_ok=true
	# Exit loop
	break
    fi
    # Unmount has failed
    echo "\033[0;31m\033[1m$(date) | Unmount failed\033[0m\033[0m"
    # Wait for some time
    sleep 2
  done
  set -e
  # Jenkins job will fail if this condition is not true
  [[ "$umount_ok" == true ]]
  echo "\033[0;31m\033[1m$(date) | Umount loop-image\033[0m\033[0m"
  losetup -D
}

set_config_var() {
  lua - "$1" "$2" "$3" <<EOF > "$3.bak"
local key=assert(arg[1])
local value=assert(arg[2])
local fn=assert(arg[3])
local file=assert(io.open(fn))
local made_change=false
for line in file:lines() do
  if line:match("^#?%s*"..key.."=.*$") then
    line=key.."="..value
    made_change=true
  end
  print(line)
end

if not made_change then
  print(key.."="..value)
end
EOF
  mv "$3.bak" "$3"
}

configure_system() {

  # STATIC
  # TEMPLATE: configure_system $IMAGE $PREFIX_PATH $ROOT_PARTITON $BOOT_PARTITION

  local BLACKLIST=/etc/modprobe.d/raspi-blacklist.conf
  local CONFIG=/boot/config.txt

  BLACKLIST=$2$BLACKLIST
  CONFIG=$2$CONFIG

  # 1. Примонитровать образ

  # https://raspberrypi.stackexchange.com/questions/13137/how-can-i-mount-a-raspberry-pi-linux-distro-image
  # mount -v -o offset=48234496 -t ext4 2017-11-29-raspbian-stretch-lite.img $PREFIX_PATH
  # mount -v -o offset=4194304,sizelimit=29360128 -t vfat 2017-11-29-raspbian-stretch-lite.img $PREFIX_PATH/boot
  #
  # fdisk -l 2017-11-29-raspbian-stretch-lite.img
  # https://www.stableit.ru/2011/05/losetup.html
  # -f     : losetup сам выбрал loop (минуя занятые)
  # -P     : losetup монтирует разделы в образе как отдельные подразделы,
  #          например /dev/loop0p1 и /dev/loop0p2
  # --show : печатает имя устройства, например /dev/loop4
  echo "\033[0;31m\033[1mMount loop-image: $1\033[0m\033[0m"
  DEV_IMAGE=$(losetup -Pf $1 --show)
  sleep 0.5

  echo "\033[0;31m\033[1mMount dirs $2 & $2/boot\033[0m\033[0m"
  mount ${DEV_IMAGE}p$ROOT_PARTITION $2
  mount ${DEV_IMAGE}p$BOOT_PARTITION $2/boot

  # 2. Изменить необходимые настройки

  #   2.1. Включить sshd
  echo "\033[0;31m\033[1mTurn on sshd\033[0m\033[0m"
  touch $2/boot/ssh

  #   2.2. Включить GPIO
  # Включено по умолчанию

  #   2.3. Включить I2C
  echo "\033[0;31m\033[1mTurn on I2C\033[0m\033[0m"

  set_config_var dtparam=i2c_arm on $CONFIG &&
    if ! [ -e $BLACKLIST ]; then
      touch $BLACKLIST
    fi
    sed $BLACKLIST -i -e "s/^\(blacklist[[:space:]]*i2c[-_]bcm2708\)/#\1/"
    sed $2/etc/modules -i -e "s/^#[[:space:]]*\(i2c[-_]dev\)/\1/"
    if ! grep -q "^i2c[-_]dev" $2/etc/modules; then
      printf "i2c-dev\n" >> $2/etc/modules
    fi

  #   2.4. Включить SPI
  echo "\033[0;31m\033[1mTurn on SPI\033[0m\033[0m"

  set_config_var dtparam=spi on $CONFIG &&
    if ! [ -e $BLACKLIST ]; then
      touch $BLACKLIST
    fi
    sed $BLACKLIST -i -e "s/^\(blacklist[[:space:]]*spi[-_]bcm2708\)/#\1/"

  #   2.5. Включить raspicam
  # Включена по умолчанию вроде как

  #   2.6. Настроить AP wifi
  #   2.7. Настроить сеть на wlan
  #   2.8. Настроить DHCPd на wlan

  # Отмонтировать образ
  umount_system $2
}


prepare_fs() {

  # STATIC
  # TEMPLATE: prepare_fs $IMAGE $SIZE

  date
  #      Удаляем старый образ
  # -f : не выводить ошибки, если файла нет
  rm -f $1
  #              Копируем origin образ
  # --progress : Вывод прогресс-бара
  rsync --progress -av $1.orig $1
  expand_image $1 $2G
  date
}

install_docker() {

  # STATIC
  # TEMPLATE: install_docker $IMAGE $PREFIX_PATH $DEV_ROOTFS $DEV_BOOT

  # https://askubuntu.com/questions/485567/unexpected-end-of-file
  mount_system $1 $2 $3 $4 << EOF
#!/bin/bash
# https://www.raspberrypi.org/blog/docker-comes-to-raspberry-pi/
curl -sSL https://get.docker.com | sh
usermod -aG docker pi
systemctl enable docker
service docker start
docker pull smirart/rpi-ros:sshd
docker run -di --restart unless-stopped -p 192.168.0.121:2202:22 -t smirart/rpi-ros:sshd
EOF
  umount_system $2
}

test_docker() {

  # STATIC
  # TEMPLATE: test_docker $IMAGE $PREFIX_PATH $DEV_ROOTFS $DEV_BOOT

  mount_system $1 $2 $3 $4 << EOF
#!/bin/bash
# https://www.raspberrypi.org/blog/docker-comes-to-raspberry-pi/
service docker start
sleep 1
docker images
docker ps -a
EOF
  umount_system $2
}

enter() {

  # STATIC
  # TEMPLATE: enter $IMAGE $PREFIX_PATH $DEV_ROOTFS $DEV_BOOT

  mount_system $1 $2 $3 $4
  umount_system $2
}

execute() {

  # STATIC
  # TEMPLATE: execute $IMAGE $PREFIX_PATH $DEV_ROOTFS $DEV_BOOT $EXECUTE_FILE ...
  mount_system2 $1 $2 $3 $4 "$5" ${@:6}
  umount_system2 $2
}


# очистить history
# https://askubuntu.com/questions/191999/how-to-clear-bash-history-completely
# cat /dev/null > ~/.bash_history && history -c && exit
#
# screen in chroot
# getty tty
# https://stackoverflow.com/questions/19104894/screen-must-be-connected-to-a-terminal/25646444
#
# docker in chroot
# service docker start
# https://forums.docker.com/t/cannot-connect-to-the-docker-daemon-is-the-docker-daemon-running-on-this-host/8925/17


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


# test_docker
# install_docker
# prepare_fs
# configure_system

case "$1" in
  enter) # enter $IMAGE $PREFIX_PATH $ROOT_PARTITION $BOOT_PARTITION
    enter $2 $3 $4 $5;;

  get_image) # get_image $BUILD_DIRECTORY $RPI_ZIP_NAME $RPI_DONWLOAD_URL $RPI_IMAGE_NAME $IMAGE_NAME
    get_image $2 $3 $4 $5 $6;;

  resize_fs) # resize_fs $SIZE $BUILD_DIRECTORY $IMAGE_NAME $ROOT_PARTITION
    resize_fs $2 $3 $4 $5;;

  publish_image) # publish_image $BUILD_DIRECTORY $IMAGE_NAME $WORKSPACE $CONFIG_FILE $RELEASE_ID $RELEASE_BODY
    publish_image $2 $3 $4 $5 $6 $7;;

  publish_image2) # publish_image2 $BUILD_DIRECTORY $IMAGE_NAME $WORKSPACE $CONFIG_FILE $RELEASE_ID $RELEASE_BODY
    publish_image2 $2 $3 $4 $5 $6 $7;;

  execute) # execute $IMAGE $PREFIX_PATH $ROOT_PARTITION $BOOT_PARTITION $EXECUTE_FILE ...
    execute $2 $3 $4 $5 $6 ${@:7};;

  *)
    echo "Enter one of: enter, get_image, resize_fs, publish_image, execute";;
esac
