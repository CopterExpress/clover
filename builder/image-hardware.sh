#! /usr/bin/env bash

set -ex

echo "Build overlay for OV7251 camera on CAM0"

dtc -I dts -O dtb -o /boot/overlays/ov7251cam0.dtbo /home/pi/catkin_ws/src/clover/builder/assets/ov7251cam0-overlay.dts
