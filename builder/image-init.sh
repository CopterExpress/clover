#! /usr/bin/env bash

#
# Script for image initialisation
#
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Artem Smirnov <urpylka@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -ex # exit on error, echo commands

echo "--- Move /etc/ld.so.preload out of the way"
mv /etc/ld.so.preload /etc/ld.so.preload.disabled-for-build

echo "--- Create pi user"
echo 'pi:$6$c70VpvPsVNCG0YR5$l5vWWLsLko9Kj65gcQ8qvMkuOoRkEagI90qi3F/Y7rm8eNYZHW8CY6BOIKwMH7a3YYzZYL90zf304cAHLFaZE0' > /boot/userconf.txt

echo "--- Write Clover information"
# Clover image version
echo "$1" >> /etc/clover_version
# Origin image file name
echo "${2%.*}" >> /etc/clover_origin

echo "--- Write magic script to /etc/rc.local"
MAGIC_SCRIPT="sudo /root/init_rpi.sh; sudo sed -i '/sudo \\\/root\\\/init_rpi.sh/d' /etc/rc.local && sudo reboot"
sed -i "19a${MAGIC_SCRIPT}" /etc/rc.local

# It needs for autosizer.sh & maybe that is correct
echo "--- Change boot partition"
sed -i 's/root=[^ ]*/root=\/dev\/mmcblk0p2/' /boot/cmdline.txt
sed -i 's/.*  \/boot\/firmware  vfat    defaults          0       2$/\/dev\/mmcblk0p1  \/boot\/firmware  vfat  defaults          0       2/' /etc/fstab
sed -i 's/.*  \/               ext4    defaults,noatime  0       1$/\/dev\/mmcblk0p2  \/               ext4    defaults,noatime  0       1/' /etc/fstab
cat /boot/cmdline.txt
cat /etc/fstab

echo "--- Set max space for syslogs"
# https://unix.stackexchange.com/questions/139513/how-to-clear-journalctl
sed -i 's/#SystemMaxUse=/SystemMaxUse=200M/' /etc/systemd/journald.conf
