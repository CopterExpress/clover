#!/bin/bash

set -e

echo -e "\033[0;31m\033[1m$(date) | #1 Change boot partition\033[0m\033[0m"

sed -i 's/root=[^ ]*/root=\/dev\/mmcblk0p2/' /boot/cmdline.txt
sed -i 's/.*  \/boot           vfat    defaults          0       2$/\/dev\/mmcblk0p1  \/boot           vfat    defaults          0       2/' /etc/fstab
sed -i 's/.*  \/               ext4    defaults,noatime  0       1$/\/dev\/mmcblk0p2  \/               ext4    defaults,noatime  0       1/' /etc/fstab

echo -e "\033[0;31m\033[1m$(date) | End of change boot partition\033[0m\033[0m"
