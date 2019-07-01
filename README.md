# CLEVER

<img src="docs/assets/clever3.png" align="right" width="300px" alt="CLEVER drone">

CLEVER (Russian: *"Клевер"*, meaning *"Clover"*) is an educational programmable drone kit consisting of an unassembled quadcopter, open source software and documentation. The kit includes Pixhawk/Pixracer autopilot running PX4 firmware, Raspberry Pi 3 as companion computer, a camera for computer vision navigation as well as additional sensors and peripheral devices.

Copter Express has implemented a large number of different autonomous drone projects using exactly the same platform: [automated pizza delivery](https://www.youtube.com/watch?v=hmkAoZOtF58) in Samara and Kazan, coffee delivery in Skolkovo Innovation Center, [autonomous quadcopter with charging station](https://www.youtube.com/watch?v=RjX6nUqw1mI) for site monitoring and security, winning drones on [Robocross-2016](https://www.youtube.com/watch?v=dGbDaz_VmYU) and [Robocross-2017](https://youtu.be/AQnd2CRczbQ) competitions and many others.

**The main documentation in Russian is available [on our Gitbook](https://clever.copterexpress.com/).**

Use it to learn how to assemble, configure, pilot and program autonomous CLEVER drone.

## Raspberry Pi image

**Preconfigured image for Raspberry Pi 3 with installed and configured software, ready to fly, is available [in the Releases section](https://github.com/CopterExpress/clever/releases).**

[![Build Status](https://travis-ci.org/CopterExpress/clever.svg?branch=master)](https://travis-ci.org/CopterExpress/clever)

Image includes:

* Raspbian Stretch
* ROS Kinetic
* Configured networking
* OpenCV
* mavros
* Periphery drivers (`pigpiod`, `rpi_ws281x`, etc)
* CLEVER software bundle for autonomous drone control

API description (in Russian) for autonomous flights is available [on GitBook](https://clever.copterexpress.com/simple_offboard.html).

## Manual installation

Install ROS Kinetic according to the [documentation](http://wiki.ros.org/kinetic/Installation), then [create a Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

Clone this repo to directory `~/catkin_ws/src/clever`:

```bash
cd ~/catkin_ws/src
git clone https://github.com/CopterExpress/clever.git clever
```

All the required ROS packages (including `mavros` and `opencv`) can be installed using `rosdep`:

```bash
cd ~/catkin_ws/
rosdep install -y --from-paths src --ignore-src
```

Build ROS packages (on memory constrained platforms you might be going to need to use `-j1` key):

```bash
cd ~/catkin_ws
catkin_make -j1
```

To complete `mavros` install you'll need to install `geographiclib` datasets:

```bash
curl https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | sudo bash
```

## Running

Enable systemd service `roscore` (if not running):

```bash
sudo systemctl enable /home/<username>/catkin_ws/src/clever/deploy/roscore.service
sudo systemctl start roscore
```

To start connection to SITL, use:

```bash
roslaunch clever sitl.launch
```

To start connection to the flight controller, use:

```bash
roslaunch clever clever.launch
```

Also, you can enable and start the systemd service:

```bash
sudo systemctl enable /home/<username>/catkin_ws/src/clever/deploy/clever.service
sudo systemctl start clever
```
