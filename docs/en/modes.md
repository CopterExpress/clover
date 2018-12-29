Flight modes
===

The flight controller mode determines the exact behavior of the copter (or another craft): how incoming commands and signals from the transmitter are to be interpreted.

PX4
---

Main article: https://dev.px4.io/en/concept/flight_modes.html

### Manual control (MANUAL)

In manual mode, the pilot controls the drone directly. GPS, computer vision data, and barometer are not used. Flying in these modes requires good drone piloting skills.

* **MANUAL** — signals from the transmitter are sent directly to the mixer and to the motors. Control of the throttle and of the front/back, right/left pairs of motors rotation speed ratio. This mode is almost never used for flying, since keeping the copter stable is too difficult.

* **STABILIZED** — the mode with stabilized horizontal position. Control of the throttle, the copter pitch and roll, and the yaw angular velocity.

* **ACRO** — control of throttle and the copter's pitch, roll, and yaw angular velocity. Used by drone racers and in 3D piloting shows for performing stunts.

* **RATTITUDE** — in the center, the right stick is similar to STABILIZED, at the edges, it passes to the ACRO mode.

### With the use of additional sensors (ASSISTED)

* **ALTCTL** (ALTITUDE) — control of the altitude rate, pitch, roll and yaw angular velocity. The barometer is used (or another height gauge).

* **POSCTL** (POSITION) — control of the altitude rate, forward/backward and right/left speed, and yaw angular velocity. It is the easiest flying mode. The barometer, GPS, computer vision, and other sensors are used.

### Automatic flight (AUTO)

In the automatic flight mode, the quadcopter ignores the control signals from the transmitter.

* **AUTO.MISSION** – PX4 completes the mission pre-loaded into the drone (the mission is downloaded using the QGroundControl, or from [MAVLink] (mavlink.md) using [MAVROS](mavros.md).
* **AUTO.RTL** – the copter automatically returns to the takeoff point.
* **AUTO.LAND** – the copter lands automatically.

### Control from an external computer

* **OFFBOARD** in the mode for flying at the MAVLink commands. Control from an external computer (e.g., Raspberry Pi).

The main used MAVLink packages are:

* [MAV_CMD_COMPONENT_ARM_DISARM](https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM)
* [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)
* [SET_ATTITUDE_TARGET](https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET)

See: [autonomous flying the quadcopter in the OFFBOARD mode](simple_offboard.md).