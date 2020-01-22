#! /usr/bin/env bash

#
# Script for build the image. Used builder script of the target repo
# For build: docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/builder/repo smirart/builder
#
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Artem Smirnov <urpylka@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -e # Exit immidiately on non-zero result

REPO=$1
REF=$2
INSTALL_ROS_PACK_SOURCES=$3
DISCOVER_ROS_PACK=$4
NUMBER_THREADS=$5

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

  TEXT="$(date '+[%Y-%m-%d %H:%M:%S]') $1"
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

# https://gist.github.com/letmaik/caa0f6cc4375cbfcc1ff26bd4530c2a3
# https://github.com/travis-ci/travis-build/blob/master/lib/travis/build/templates/header.sh
my_travis_retry() {
  local result=0
  local count=1
  local max_count=50
  while [ $count -le $max_count ]; do
    [ $result -ne 0 ] && {
      echo -e "\nThe command \"$@\" failed. Retrying, $count of $max_count.\n" >&2
    }
    # ! { } ignores set -e, see https://stackoverflow.com/a/4073372
    ! { "$@"; result=$?; }
    [ $result -eq 0 ] && break
    count=$(($count + 1))
    sleep 1
  done

  [ $count -gt $max_count ] && {
    echo -e "\nThe command \"$@\" failed $max_count times.\n" >&2
  }

  return $result
}

# TODO: 'kinetic-rosdep-clover.yaml' should add only if we use our repo?
echo_stamp "Init rosdep"
my_travis_retry rosdep init
<<<<<<< HEAD
echo "yaml file:///etc/ros/rosdep/melodic-rosdep-clover.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list
=======
echo "yaml file:///etc/ros/rosdep/melodic-rosdep-clever.yaml" >> /etc/ros/rosdep/sources.list.d/20-default.list
>>>>>>> master
my_travis_retry rosdep update

echo_stamp "Populate rosdep for ROS user"
my_travis_retry sudo -u pi rosdep update

resolve_rosdep() {
  # TEMPLATE: resolve_rosdep <CATKIN_PATH> <ROS_DISTRO> <OS_DISTRO> <OS_VERSION>
  CATKIN_PATH=$1
  ROS_DISTRO='melodic'
  OS_DISTRO='debian'
  OS_VERSION='buster'

  echo_stamp "Installing dependencies apps with rosdep in ${CATKIN_PATH}"
  cd ${CATKIN_PATH}
  my_travis_retry rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --os=${OS_DISTRO}:${OS_VERSION}
}

export ROS_IP='127.0.0.1' # needed for running tests

echo_stamp "Reconfiguring Clover repository for simplier unshallowing"
cd /home/pi/catkin_ws/src/clover
git config remote.origin.fetch "+refs/heads/*:refs/remotes/origin/*"

echo_stamp "Make clever package for backwards compatibility"
cd /home/pi/catkin_ws
mkdir -p src/clever/clever/srv src/clever/clever/launch
cd src/clever/clever/srv && ln -s ../../../clover/clover/srv/* ./
cd ../launch && ln -s ../../../clover/clover/launch/* ./
ln -s clover.launch clever.launch
cd /home/pi/catkin_ws
cp src/clover/builder/assets/clever/CMakeLists.txt src/clever/clever/
cp src/clover/builder/assets/clever/_package.xml src/clever/clever/package.xml

echo_stamp "Build and install Clover"
resolve_rosdep $(pwd)
my_travis_retry pip install wheel
my_travis_retry pip install -r /home/pi/catkin_ws/src/clover/clover/requirements.txt
source /opt/ros/melodic/setup.bash
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release

echo_stamp "Add deprecation warning to clever srv files"
cp -r src/clover/builder/assets/clever/srv /home/pi/catkin_ws/devel/lib/python2.7/dist-packages/clever/

echo_stamp "Enable ROS services"
systemctl enable roscore
systemctl enable clover

echo_stamp "Build Clover documentation"
cd /home/pi/catkin_ws/src/clover
NPM_CONFIG_UNSAFE_PERM=true npm install gitbook-cli -g
NPM_CONFIG_UNSAFE_PERM=true gitbook install
gitbook build
touch node_modules/CATKIN_IGNORE docs/CATKIN_IGNORE _book/CATKIN_IGNORE clover/www/CATKIN_IGNORE apps/CATKIN_IGNORE # ignore documentation files by catkin

echo_stamp "Installing additional ROS packages"
apt-get install -y --no-install-recommends \
    ros-melodic-dynamic-reconfigure \
    ros-melodic-compressed-image-transport \
    ros-melodic-rosbridge-suite \
    ros-melodic-rosserial \
    ros-melodic-usb-cam \
    ros-melodic-vl53l1x \
    ros-melodic-ws281x \
    ros-melodic-rosshow

# TODO move GeographicLib datasets to Mavros debian package
echo_stamp "Install GeographicLib datasets (needed for mavros)" \
&& wget -qO- https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash

# FIXME: Buster comes with tornado==5.1.1 but we need tornado==4.2.1 for rosbridge_suite
# (note that Python 3 will still have a more recent version)
pip install tornado==4.2.1

echo_stamp "Running tests"
cd /home/pi/catkin_ws
# FIXME: Investigate failing tests
catkin_make run_tests #&& catkin_test_results

echo_stamp "Change permissions for catkin_ws"
chown -Rf pi:pi /home/pi/catkin_ws

echo_stamp "Setup ROS environment"
cat << EOF >> /home/pi/.bashrc
LANG='C.UTF-8'
LC_ALL='C.UTF-8'
ROS_DISTRO='melodic'
export ROS_HOSTNAME=\`hostname\`.local
source /opt/ros/melodic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
EOF

#echo_stamp "Removing local apt mirror"
# Restore original sources.list
#mv /var/sources.list.bak /etc/apt/sources.list
# Clean apt cache
apt-get clean -qq > /dev/null
# Remove local mirror repository key
#apt-key del COEX-MIRROR

echo_stamp "END of ROS INSTALLATION"
