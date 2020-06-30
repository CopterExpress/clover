# COEX Clover Drone Kit

<table align=center>
    <tr>
        <td align=center><img src="docs/assets/px4.svg" height=100></td>
        <td align=center><big>+</big></td>
        <td align=center><img src="docs/assets/rpi.svg" height=100></td>
        <td align=center><big>+</big></td>
        <td align=center><img src="docs/assets/ros.svg" height=100></td>
    </tr>
</table>

This repository contains documentation, software platform source and RPi image builder for COEX Clover.

<img src="docs/assets/clover42.png" align="right" width="400px" alt="COEX Clover">

Clover is a [PX4](https://px4.io)- and [ROS](https://www.ros.org)-powered educational programmable drone kit consisting of an unassembled quadcopter, open source software and documentation. The kit includes Pixracer-compatible autopilot, Raspberry Pi 4 as companion computer, a camera for computer vision navigation as well as additional sensors and peripheral devices.

The main documentation is available at [https://clover.coex.tech](https://clover.coex.tech/).

Official website: <a href="https://coex.tech/clover">coex.tech/clover</a>.

## Autonomous flights compilation

[![Clover Drone Kit autonomy compilation](http://img.youtube.com/vi/u3omgsYC4Fk/hqdefault.jpg)](https://youtu.be/u3omgsYC4Fk)

Clover drone is used on a wide range of educational events, including [Copter Hack](https://www.youtube.com/watch?v=xgXheg3TTs4), WorldSkills Drone Operation competition, [Autonomous Vehicles Track of NTI Olympics 2016–2020](https://www.youtube.com/watch?v=E1_ehvJRKxg), Quadro Hack 2019 (National University of Science and Technology MISiS), Russian Robot Olympiad (autonomous flights), and others.

## PX4 Dev Summit 2019 talk

[![](http://img.youtube.com/vi/CTG9E9PbJQ8/0.jpg)](http://www.youtube.com/watch?v=CTG9E9PbJQ8)

## Features

### Prebuilt RPi image

...

### Common robotics software

ROS, OpenCV...

### QGroundControl Wi-Fi bridge

...

### Easy autonomous flights programming

Taking off, navigating and land is just:

```python
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)  # takeoff and hover 1 m above the ground
```

```python
navigate(x=1, y=0, z=0, frame_id='body')  # fly forward 1 m
```

```
land()
```

See [programming documentation](https://clover.coex.tech) for further information.

### Optical flow positioning

RPi based optical flow....

### ArUco markers recognizing

...

### Easy work with peripheral devices

Preinstalled package for the [LED strip](https://clover.coex.tech/en/leds.html) allows high-level control (such as rainbow effect or color fade) as well as individual LED low-level control:

```python
set_effect(r=0, g=100, b=0)  # fill strip with green color
```

```python
set_effect(effect='fade', r=0, g=0, b=255)  # fade to blue color
```

```python
set_effect(effect='rainbow')  # show rainbow
```

Preinstalled [VL53L1X rangefinder driver](https://clover.coex.tech/en/laser.html) passes data to the flight controller automatically and allows the user to get its data:

```python
data = rospy.wait_for_message('rangefinder/range', Range)  # get data from the rangefinder
```

Preinstalled fast Python [GPIO library](https://clover.coex.tech/en/gpio.html).

```python
pi.write(11, 1)  # set signal of pin 11 to high
```

```python
level = pi.read(12)  # read the state of pin 12
```

### Simulator

Gazebo-based simulation.

Screenshot...

### Remote control apps

...

### Community

Community photo...

Clover is widely used ...

### Free and open source

The Clover software bundle is free, open source, and compatible with any PX4-based drone.

## Raspberry Pi image

Preconfigured image for Raspberry Pi with installed and configured software, ready to fly, is available [in the Releases section](https://github.com/CopterExpress/clover/releases).

[![Build Status](https://travis-ci.org/CopterExpress/clover.svg?branch=master)](https://travis-ci.org/CopterExpress/clover)

Image features:

* Raspbian Buster
* [ROS Melodic](http://wiki.ros.org/melodic)
* Configured networking
* OpenCV
* [`mavros`](http://wiki.ros.org/mavros)
* Periphery drivers for ROS ([GPIO](https://clover.coex.tech/en/gpio.html), [LED strip](https://clover.coex.tech/en/leds.html), etc)
* `aruco_pose` package for marker-assisted navigation
* `clover` package for autonomous drone control

API description for autonomous flights is available [on GitBook](https://clover.coex.tech/en/simple_offboard.html).

For manual package installation and running see [`clover` package documentation](clover/README.md).

## Other resources

* Official documentation: [https://clover.coex.tech](https://clover.coex.tech).
* ROS Wiki page: [https://wiki.ros.org/Robots/clover](https://wiki.ros.org/Robots/clover).
* ROS Robots page: [https://robots.ros.org/clover](https://robots.ros.org/clover).

## License

While the Clover platform source code is available under the MIT License, note, that the [documentation](docs/) is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
