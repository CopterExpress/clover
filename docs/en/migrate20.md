# Migration to version 0.20

[Image](image.md) version v0.20 includes significant changes in comparison with the version 0.19. When transitioning please nott changes presented below.

## ROS package `clever` is renamed to `clover`

All the imports in Python scripts should be changed.

Before:

```python
# coding: utf8

import rospy
from clever import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Take off 1 m
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
```

After:

```python
import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Take off 1 м
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
```

## systemd service `clever` is renamed to `clover`

For restarting the platform instead of:

```bash
sudo systemctl restart clever
```

use command:

```bash
sudo systemctl restart clover
```

## Path to platform's files changed

The `~/catkin_ws/src/clever/` directory is renamed to `~/catkin_ws/src/clover`. Thus, configuration files (`.launch`) are to be edited using the new path.

For example, `~/catkin_ws/src/clever/clever/launch/clever.launch` file is now `~/catkin_ws/src/clover/clover/launch/clover.launch`.

<!--
## Python 3 transition

Python 2 is depracated since, January 1st, 2020. The Clover platform moves to Python 3.

For running flight script instead of `python` command:

```bash
python flight.py
```

use `python3` command:

```bash
python3 flight.py
```

Python 3 has certain syntax differences in comparison with the old version. Instead of `print` *operator*:

```python
print 'Clover is the best'
```

use `print` *function*:

```python
print('Clover is the best')
```

The division operator operates floating points by default (instead of integer). Python 2:

```python
>>> 10 / 4
2
```

Python 3:

```python
>>> 10 / 4
2.5
```

For strings `unicode` type is used by default (instead of `str` type).

Encoding specification (`# coding: utf8`) is not necessary any more.

More details on all the language changes see in [appropriate article](https://sebastianraschka.com/Articles/2014_python_2_3_key_diff.html).
-->

## Wi-Fi network configuration

Wi-Fi networks' SSID is changed to `clover-XXXX` (where X is a random number), password is changed to `cloverwifi`.

## The camera orientation configuration changed

See details in the "[Camera setup](camera_setup.md#frame)" article.
