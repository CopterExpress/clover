#!/bin/bash

set -e

##################################################################################################################################
# Настройка интерфейсов
##################################################################################################################################

# вот так все в принципе должно включиться
# /usr/bin/raspi-config nonint do_i2c 0
# /usr/bin/raspi-config nonint do_spi 0
# /usr/bin/raspi-config nonint do_camera 0
# /usr/bin/raspi-config nonint do_rgpio 0
# /usr/bin/raspi-config nonint do_ssh 0

# по идеи эти настройки должны проводиться до по другому как сделано в prepare_image.sh

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

BLACKLIST=/etc/modprobe.d/raspi-blacklist.conf
CONFIG=/boot/config.txt

# 2. Изменить необходимые настройки

#   2.1. Включить sshd
echo -e "\033[0;31m\033[1m$(date) | #1 Turn on sshd\033[0m\033[0m"
touch /boot/ssh

#   2.2. Включить GPIO
# Включено по умолчанию

#   2.3. Включить I2C
echo -e "\033[0;31m\033[1m$(date) | #2 Turn on I2C\033[0m\033[0m"

set_config_var dtparam=i2c_arm on $CONFIG &&
  if ! [ -e $BLACKLIST ]; then
    touch $BLACKLIST
  fi
  sed $BLACKLIST -i -e "s/^\(blacklist[[:space:]]*i2c[-_]bcm2708\)/#\1/"
  sed /etc/modules -i -e "s/^#[[:space:]]*\(i2c[-_]dev\)/\1/"
  if ! grep -q "^i2c[-_]dev" /etc/modules; then
    printf "i2c-dev\n" >> /etc/modules
  fi

#   2.4. Включить SPI
echo -e "\033[0;31m\033[1m$(date) | #3 Turn on SPI\033[0m\033[0m"

set_config_var dtparam=spi on $CONFIG &&
  if ! [ -e $BLACKLIST ]; then
    touch $BLACKLIST
  fi
  sed $BLACKLIST -i -e "s/^\(blacklist[[:space:]]*spi[-_]bcm2708\)/#\1/"

#   2.5. Включить raspicam
echo -e "\033[0;31m\033[1m$(date) | #4 Turn on raspicam\033[0m\033[0m"

get_config_var() {
  lua - "$1" "$2" <<EOF
local key=assert(arg[1])
local fn=assert(arg[2])
local file=assert(io.open(fn))
local found=false
for line in file:lines() do
  local val = line:match("^%s*"..key.."=(.*)$")
  if (val ~= nil) then
    print(val)
    found=true
    break
  end
end
if not found then
   print(0)
end
EOF
}

# тут уже немного иначе, но по сути одно и тоже
# https://github.com/RPi-Distro/raspi-config/blob/master/raspi-config#L1136
# $1 is 0 to disable camera, 1 to enable it
set_camera() {
  # Stop if /boot is not a mountpoint
  #if ! mountpoint -q /boot; then
  #  return 1
  #fi

  [ -e $CONFIG ] || touch $CONFIG

  if [ "$1" -eq 0 ]; then # disable camera
    set_config_var start_x 0 $CONFIG
    sed $CONFIG -i -e "s/^startx/#startx/"
    sed $CONFIG -i -e "s/^start_file/#start_file/"
    sed $CONFIG -i -e "s/^fixup_file/#fixup_file/"
  else # enable camera
    set_config_var start_x 1 $CONFIG
    CUR_GPU_MEM=$(get_config_var gpu_mem $CONFIG)
    if [ -z "$CUR_GPU_MEM" ] || [ "$CUR_GPU_MEM" -lt 128 ]; then
      set_config_var gpu_mem 128 $CONFIG
    fi
    sed $CONFIG -i -e "s/^startx/#startx/"
    sed $CONFIG -i -e "s/^fixup_file/#fixup_file/"
  fi
}

if [ ! -e /boot/start_x.elf ];
then echo "Your firmware appears to be out of date (no start_x.elf). Please update"
else set_camera 1
fi

# Включение V4L драйвера http://robocraft.ru/blog/electronics/3158.html
#echo "bcm2835-v4l2" >> /etc/modules
if ! grep -q "^bcm2835-v4l2" /etc/modules; then
  printf "bcm2835-v4l2\n" >> /etc/modules
fi

echo -e "\033[0;31m\033[1m$(date) | #5 End of configuring interfaces\033[0m\033[0m"
