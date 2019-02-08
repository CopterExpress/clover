# Glossary

## Quadcopter

An unmanned aerial vehicle with 4 propellers and an electronic stabilization system.

## Multicopter

An unmanned aerial vehicle with an electronic stabilization system and the number of propellers equal to 3 (tricopter), 4 (quadcopter), 6 (hexacopter), 8 (octocopter), or more.

## Flight controller / autopilot

**1\.** A specialized circuit-board designed for controlling a multicopter, an aircraft or another apparatus. Examples:
Pixhawk, Ardupilot, Naze32, CC3D.

**2\.** Software for the multicopter control circuit-board. Examples: PX4, APM, CleanFlight.

## Motor

An electric motor that rotates propellers of the multicopter. Usually, brushless motors are used. These motors are connected to ESC.

## ESC / motor controller

An Electronic Speed Controller. A specialized circuit-board that controls the speed of the brushless motor. It is controlled by a flight controller using pulse width modulation (PWM).

ESC has the firmware that determines the characteristics of its operation.

## Remote control / radio control equipment

A radio-operated quadcopter remote control. Operation of the remote control requires connecting a receiver to the flight controller.

Clever may also be [controlled from a smartphone](rc.md).

## Telemetry

**1\.** Transmitting the data about the state of a quadcopter or another aircraft over a distance.

**2\.** The sum of data about the aircraft state as such (height, orientation, global coordinates, etc.).

**3\.** A system for transmitting the data about the  aircraft state or commands to it over the air. Examples: radio modems (RFD900, 3DR Radio Modem), Wi-Fi modules (ESP-07). Raspberry Pi may also be used in Clever as a telemetry module: [the use of QGroundControl via Wi-Fi](gcs_bridge.md).

## Arming

Armed is the state of copter readiness for the fligh. When the gas stick is lifted, or when an external command with the target point is sent, the copter will fly. Usually, a copter starts rotating its propellers when it is switched to the "armed" state, even if the gas stick is down.

The opposite state is Disarmed.

## PX4

A popular open source flight controller that works with circuit-boards Pixhawk, Pixracer, and others. PX4 is recommended to be used with Clever.

## APM / ArduPilot

An open source flight controller originally created for the Arduino circuit-boards. It was later ported to Pixhawk, Pixracer and other boards.

## MAVLink

A communication protocol for drones, ground stations and other devices over the radio channels. Usually, it is this protocol that is used for telemetry.

## ROS

A popular framework for writing complex robotics applications.

## MAVROS

A library that is a link between the aircraft operating using the MAVLink protocol, and ROS.

## UART

A serial asynchronous data transfer interface used in many devices. For example, GPS antennas, Wi-Fi routers, or Pixhawk.
