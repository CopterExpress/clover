Simple OFFBOARD
===

> **Note** Documentation for the [image](microsd_images.md), versions, starting with **0.15**. For older versions refer to [documentation for version **0.14**](https://github.com/droneExpress/clever/blob/v0.14/docs/ru/simple_offboard.md).

<!-- -->

> **Hint** For autonomous flights it is recommanded to use [special firmware PX4 for Clever](firmware.md#прошивка-для-клевера).

The `simple_offboard` module of the `clever` package is intended for simplified programming of the autonomous drone ([mode](modes.md) `OFFBOARD`). It allows setting the desired flight tasks, and automatically transforms [the system of coordinates](frames.md).

`simple_offboard` is a high level way of interacting with the flight controller. For a more low level work, see [mavros](mavros.md).

Main services are `get_telemetry` (receiving all telemetry), `navigate` (flying to a given point along a straight line), `navigate_global` (flying to a global point along a straight line), `land` (switching to the landing mode).

The use of Python language
---

To use the services, create proxies to them. Following is an example of the program that declares proxies to all `simple_offboard` services:

```python
import rospy
from clever import srv
from std_srvs.srv import Trigger

rospy.init_node('flight') # flight – name of your ROS node

# Creating proxies to all services:

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
```

Unused proxy functions may be removed from the code.

API description
---

> **Note** Blank numeric parameters are set to 0.

### get_telemetry

Obtaining complete telemetry of the drone.

Parameters:

* `frame_id` – [frame](frames.md) for values `x`, `y`, `z`, `vx`, `vy`, `vz`. Example: `map`, `body`, `aruco_map`. Default value: `map`.

Response format:

* `frame_id` — frame;
* `connected` – whether there is a connection to <abbr title="Flight Control Unit flight controller">FCU</abbr>;
* `armed` - state of propellers (the propellers are armed, if true);
* `mode` – current [flight mode](modes.md);
* `x, y, z` — local position of the drone *(m)*;
* `lat, lon` – latitude, longitude *(degrees)*, [GPS](gps.md) is to be available;
* `alt` – altitude in the global system of coordinates (standard [WGS-84](https://ru.wikipedia.org/wiki/WGS_84), not <abbr title="Above Mean Sea Level">AMSL</abbr>!), [GPS](gps.md) is to be available ;
* `vx, vy, vz` – drone velocity *(m/s)*;
* `pitch` – pitch angle *(radians)*;
* `roll` – roll angle *(radians)*;
* `yaw` — yaw angle *(radians)*;
* `pitch_rate` — angular pitch velocity *(rad/s)*;
* `roll_rate` – angular roll velocity *(rad/s)*;
* `yaw_rate` – angular yaw velocity *(rad/s)*;
* `voltage` – total battery voltage *(V)*;
* `cell_voltage` – battery cell voltage *(V)*.

> **Note** Fields that are unavailabe for any reason will contain the `NaN` value.

Displaying drone coordinates `x`, `y` and `z` in the local system of coordinates:

```python
telemetry = get_telemetry()
print telemetry.x, telemetry.y, telemetry.z
```

Displaying drone altitude relative to [the card of ArUco tags](aruco.md):

```python
telemetry = get_telemetry(frame_id='aruco_map')
print telemetry.z
```

Checking global position availability:

```python
import math
if not math.isnan(get_telemetry().lat):
    print 'Global position presents'
else:
    print 'No global position'
```

Output of current telemetry (command line):

```(bash)
rosservice call /get_telemetry "{frame_id: ''}"
```

### navigate

Fly to the designated point in a straight line.

Parameters:

* `x`, `y` — coordinates *(m)*;
* `yaw` — yaw angle *(radians)*;
* `yaw_rate` – angular yaw velocity (used for setting the yaw to `NaN`) *(rad/s)*;
* `speed` – flight speed (setpoint speed) *(m/s)*;
* `auto_arm` – switch the drone to `OFFBOARD` and arm automatically (**the drone will take off**);
* `frame_id` – [system of coordinates](frames.md) for values `x`, `y`, `z`, `vx`, `vy`, `vz`. Example: `map`, `body`, `aruco_map`. Default value: `map`.

> **Note** To fly without changing the yaw angle, it is sufficient to set the `yaw` to `NaN` (angular velocity by default is 0).

Ascending to the altitude of 1.5 m with the climb rate of 0.5 m/s:

```python
navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
```

Flying in a straight line to point 5:0 (altitude 2) in the local system of coordinates at the speed of 0.8 m/s (yaw is set to 0):

```python
navigate(x=5, y=0, z=3, speed=0.8)
```

Flying to point 5:0 without changing the yaw angle (`yaw` = `NaN`, `yaw_rate` = 0):

```python
navigate(x=5, y=0, z=3, speed=0.8, yaw=float('nan'))
```

Flying 3 m to the right from the drone:

```python
navigate(x=0, y=-3, z=0, speed=1, frame_id='body')
```

Turn 90 degrees counterclockwise:

```python
navigate(yaw=math.radians(-90), frame_id='body')
```

Flying to point 3:2 (altitude 2) in the system of coordinates [of the marker field](aruco.md) at the speed of 1 m/s:

```python
navigate(x=3, y=2, z=2, speed=1, frame_id='aruco_map')
```

Rotating on the spot at the speed of 0.5 rad/s (counterclockwise):

```python
navigate(x=0, y=0, z=0, yaw=float('nan'), yaw_rate=0.5, frame_id='body')
```

Flying 3 meters forwards at the speed of 0.5 m/s, yaw-rotating at the speed of 0.2 rad/s:

```python
navigate(x=3, y=0, z=0, speed=0.5, yaw=float('nan'), yaw_rate=0.2, frame_id='body')
```

Ascending to the altitude of 2 m (command line):

```(bash)
rosservice call /navigate "{x: 0.0, y: 0.0, z: 2, yaw: 0.0, yaw_rate: 0.0, speed: 0.5, frame_id: 'body', auto_arm: true}"
```

### navigate_global

Flying in a straight line to a point in the global system of coordinates (latitude/longitude).

Parameters:

* `lat`, `lon` — latitude and longitude *(degrees)*;
* `z` — altitude *(m)*;
* `yaw` — yaw angle *(radians)*;
* `yaw_rate` – angular yaw velocity (used for setting the yaw to `NaN`) *(rad/s)*;
* `speed` – flight speed (setpoint speed) *(m/s)*;
* `auto_arm` – switch the drone to `OFFBOARD` and arm automatically (**the drone will take off**);
* `frame_id` – [system of coordinates](frames.md), given `z` и `yaw` (Default value: `map`).

> **Note** To fly without changing the yaw angle, it is sufficient to set the `yaw` to `NaN` (angular velocity by default is 0).

Flying to a global point at the speed of 5 m/s, while remaining at current altitude (`yaw` will be set to 0, the drone will face East):

```python
navigate_global(lat=55.707033, lon=37.725010, z=0, speed=5, frame_id='body')
```

Flying to a global point without changing the yaw angle (`yaw` = `NaN`, `yaw_rate` = 0):

```python
navigate_global(lat=55.707033, lon=37.725010, z=0, speed=5, yaw=float('nan'), frame_id='body')
```

Flying to a global point (command line):

```(bash)
rosservice call /navigate_global "{lat: 55.707033, lon: 37.725010, z: 0.0, yaw: 0.0, yaw_rate: 0.0, speed: 5.0, frame_id: 'body', auto_arm: false}"
```

### set_position

Set the target for position and yaw. This service may be used to specify the continuous flow of target points, for example, for flying along complex trajectories (circular, arcuate, etc.).

> **Hint** For flying to a point in a straight line or takeoff, use the [`navigate`] higher-level service (#navigate).

Parameters:

* `x`, `y`, `z` — point coordinates *(m)*;
* `yaw` — yaw angle *(radians)*;
* `yaw_rate` – angular yaw velocity (used for setting the yaw to NaN) *(rad/s)*;
* `auto_arm` – switch the drone to `OFFBOARD` and arm automatically (**the drone will take off**);
* `frame_id` – [system of coordinates](frames.md), given `x`, `y`, `z` и `yaw` (Default value: `map`).

Hovering on the spot:

```python
set_position(frame_id='body')
```

Assigning the target point 3 m above the current position:

```python
set_position(x=0, y=0, z=3, frame_id='body')
```

Assigning the target point 1 m ahead of the current position:

```python
set_position(x=1, y=0, z=0, frame_id='body')
```

Rotating on the spot at the speed of 0.5 rad/s:

```python
set_position(x=0, y=0, z=0, frame_id='body', yaw=float('nan'), yaw_rate=0.5)
```

### set_velocity

Setting speed and yaw.

* `vx`, `vy`, `vz` – required flight speed *(m/s)*;
* `yaw` — yaw angle *(radians)*;
* `yaw_rate` – angular yaw velocity (used for setting the yaw to NaN) *(rad/s)*;
* `auto_arm` – switch the drone to `OFFBOARD` and arm automatically (**the drone will take off**);
* `frame_id` – [system of coordinates](frames.md), given `vx`, `vy`, `vz` и `yaw` (Default value: `map`).

> **Note** Parameter `frame_id` specifies only the orientation of the resulting velocity vector, but not its length.

Flying forward (relative to the drone) at the speed of 1 m/s:

```python
set_velocity(vx=1, vy=0.0, vz=0, frame_id='body')
```

One of variants of flying in a circle:

```python
set_velocity(vx=0.4, vy=0.0, vz=0, yaw=float('nan'), yaw_rate=0.4, frame_id='body')
```

### set_attitude

Setting pitch, roll, yaw and throttle level (approximate analogue to control in [the `STABILIZED` mode](modes.md)). This service may be used for lower level monitoring of the drone behavior or controlling the drone, if no reliable data on its position are available.

Parameters:

* `pitch`, `roll`, `yaw` – required pitch, roll, and yaw angle *(radians)*;
* `thrust` — throttle level from 0 (no throttle, propellers are stopped) to 1 (full throttle).
* `auto_arm` – switch the drone to `OFFBOARD` and arm automatically (**the drone will take off**);
* `frame_id` – [system of coordinates](frames.md), given `yaw` (Default value: `map`).

### set_rates

Setting pitch, roll, and yaw angular velocity and the throttle level (approximate analogue to control in [the `ACRO` mode](modes.md)). This is the lowest drone control level (excluding direct control of motor rotation speed). This service may be used to automatically perform acrobatic tricks (e.g., flips).

Parameters:

* `pitch_rate`, `roll_rate`, `yaw_rate` – angular pitch, roll, and yaw velocity *(rad/s)*;
* `thrust` — throttle level from 0 (no throttle, propellers are stopped) to 1 (full throttle).
* `auto_arm` – switch the drone to `OFFBOARD` and arm automatically (**the drone will take off**);

### land

Transfer the drone to the landing [mode](modes.md) (`AUTO.LAND` or similar).

> **Note** For automatic propeller disabling after landing, [parameter PX4](px4_parameters.md) `COM_DISARM_LAND` is to be set to a value > 0.

Landing the drone:

```python
res = land()

if res.success:
    print 'drone is landing'
```

Landing the drone (command line):

```(bash)
rosservice call /land "{}"
```

<!--
### release

Stop publishing setpoints to the drone (release control). Required to continue monitoring by means of [MAVROS](mavros.md).
-->

Additional materials
------------------------

* [Flying in the field of ArUco markers](aruco.md).
* [Samples of programs and snippets](snippets.md).
