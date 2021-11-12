# Native setup

Setting up the simulation environment from scratch requires some effort, but results in the most performant setup, with less chance of driver issues.

> **Hint** See up-to-date commands set for installation Clover simulation software in the script, that builds the virtual machine image with the simulator: [`install_software.sh`](https://github.com/CopterExpress/clover_vm/blob/master/scripts/install_software.sh).

Prerequisites: Ubuntu 20.04 and [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).

## Create a workspace for the simulation

Throughout this guide we will be using the `catkin_ws` as the workspace name. Feel free to change it in your setup. We will be creating it in the home directory of the current user (`~`).

Create the workspace and clone Clover sources:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --depth 1 https://github.com/CopterExpress/clover
git clone --depth 1 https://github.com/CopterExpress/ros_led
git clone --depth 1 https://github.com/ethz-asl/mav_comm
```

Install all prerequisites using `rosdep`:

```bash
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
```

Install Python-dependencies:

```bash
sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/clover/clover/requirements.txt
```

## Get PX4 sources

PX4 will be built along with the other packages in our workspace. You may clone it directly into the workspace or put it somewhere and symlink to `~/catkin_ws/src`. We will need to put its `sitl_gazebo` submodule in `~/catkin_ws/src` as well. For simplicity's sake we will clone the firmware directly to the workspace:

```bash
cd ~/catkin_ws/src
git clone --recursive --depth 1 --branch v1.12.0 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
ln -s ~/PX4-Autopilot ~/catkin_ws/src/PX4-Autopilot
ln -s ~/PX4-Autopilot/Tools/sitl_gazebo ~/catkin_ws/src/sitl_gazebo
```

## Install PX4 prerequisites

PX4 comes with its own script for dependency installation. We may as well leverage it:

```bash
cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup
sudo ./ubuntu.sh
```

This will install everything required to build PX4 and its SITL environment.

You may want to skip installing the ARM toolchain if you're not planning on compiling PX4 for your flight controller. To do this, use the `--no-nuttx` flag:

```
sudo ./ubuntu.sh --no-nuttx
```

## Add the Clover airframe

Add the Clover airframe to PX4 using the command:

```bash
ln -s "$(catkin_find clover_simulation airframes)"/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
```

## Install geographiclib datasets

`mavros` package requires geographiclib datasets to be present:

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

## Build the simulator

With all dependencies installed, you can build your workspace:

```bash
cd ~/catkin_ws
catkin_make
```

> **Note** Some of the files - particularly Gazebo plugins - require large amounts of RAM to be built. You may wish to reduce the number of parallel jobs; the number of parallel jobs should be equal to the amount of RAM in gigabytes divided by 2 - so a 16GB machine should use no more than 8 jobs. You can specify the number of jobs using the `-j` flag: `catkin_make -j8`

## Run the simulator

In order to be sure that everything was built correctly, try running the simulator for the first time:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch clover_simulation simulator.launch
```
