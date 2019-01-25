# MAVROS

Main documentation: [http://wiki.ros.org/mavros](http://wiki.ros.org/mavros)

MAVROS \(MAVLink + ROS\) is a package for ROS that provides the possibility of controlling drones via the [MAVLink] protocol (mavlink.md). MAVROS supports flight stacks PX4 and APM. Communication is established via UART, USB, TCP or UDP.

MAVROS subscribes to certain ROS topics while waiting for commands, publishes telemetry to other topics, and provides services.

The MAVROS node is automatically started in the launch-file of Clever. For [setting the type of connection] (connection.md), see the `fcu_conn` argument.

> **Hint** Simplified interaction with the copter may be with the use of the [`simple_offboard`] package (simple_offboard.md).

<!-- -->

> **Note** In the `clever` package, some MAVROS plugins are disabled (to save resources). For more information, see the `plugin_blacklist` parameter in file `/home/pi/catkin_ws/src/clever/clever/launch/mavros.launch`.

## Main services

`/mavros/set_mode` — set [flight mode](modes.md) of the controller. Usually, the OFFBOARD mode is set \(for control from Raspberry Pi\).

`/mavros/cmd/arming` — enable or disable drone motors \\ (change the armed status \\).

## Main published topics

`/mavros/state` — status of connection to the flight controller. The flight controller mode.

`/mavros/local_position/pose` — local position of the copter in the ENU system of coordinates, and its orientation.

`/mavros/local_position/velocity` — current speed in the local coordinates. Angular velocities.

`/mavros/global_position/global` — the current global position \(latitude, longitude, altitude\).

`/mavros/global_position/local` — the global position in the [UTM] system of coordinates (https://ru.wikipedia.org/wiki/Система_координат_UTM).

`/mavros/global_position/rel_alt` — relative altitude \(relative to the engines ON altitude\).

Messages published in the topics may be viewed by using the `rostopic` utility, e.g., `rostopic echo /mavros/state`. See more in [working with ROS](ros.md).

## Main topics for publication

`/mavros/setpoint_position/local` — set the target position and the yaw of the drone \(in the ENU system of coordinates\).

`/mavros/setpoint_position/cmd_vel` — set the target linear velocity of the drone.

`/mavros/setpoint_attitude/attitude` and `/mavros/setpoint_attitude/att_throttle` — set target attitude and throttle level.

`/mavros/setpoint_attitude/cmd_vel` and `/mavros/setpoint_attitude/att_throttle` — set target angular velocity and throttle level.

### Topics for sending raw packets

`/mavros/setpoint_raw/local` — sending packet [SET\_POSITION\_TARGET\_LOCAL\_NED](https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_LOCAL_NED). Allows setting target position/target speed and target yaw/angular yaw velocity. The values to be set are selected using the `type_mask` field.

`/mavros/setpoint_raw/attitude` — sending packet [SET\_ATTITUDE\_TARGET](https://pixhawk.ethz.ch/mavlink/#SET_ATTITUDE_TARGET). Allows setting the target attitude /angular velocity and throttle level. The values to be set are selected using the `type_mask` field

`/mavros/setpoint_raw/global` — sending packet [SET\_POSITION\_TARGET\_GLOBAL\_INT](https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT). Allows setting the target attitude in global coordinates \(latitude, longitude, altitude\) and flight speed. **Not supported in PX4** \([issue](https://github.com/PX4/Firmware/issues/7552)\).