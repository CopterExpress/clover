Code examples
===

Python
---

> **Note** If Cyrillic characters in the UTF-8 charset are used, specify the charset at the beginning of the program:
> ```python
> # -*- coding: utf-8 -*-
> ```

### # {#distance}

The function of determining the distance between two points (**important**: the points are to be in the same [system of coordinates](frames.md)):

```python
def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
```

### # {#distance-global}

A function for approximate determination of the distance (in meters) between two global coordinates (latitude/longitude):

```python
def get_distance_global(lat1, lon1, lat2, lon2):
    return math.hypot(lat1 - lat2, lon1 - lon2) * 1.113195e5
```

### # {#block-takeoff}

Takeoff and waiting for the end of takeoff:

```python
z = 2  # altitude
tolerance = 0.2 # precision of altitude check (m)

# Remembering the initial point
start = get_telemetry()

# Take off to the altitude of 2 m
print navigate(z=z, speed=0.5, frame_id='body', auto_arm=True)

# Waiting for takeoff
while True:
    # Checking current altitude
    if get_telemetry().z - start.z + z < tolerance:
        # Takeoff complete
        break
    rospy.sleep(0.2)
```

### # {#block-nav}

Flying to a point and waiting for the copter to arrive at it:

```python
tolerance = 0.2 # precision of arrival check (m)
frame_id='aruco_map'

# Flying to point 1:2:3 in the field of ArUco markers
print navigate(frame_id=frame_id, x=1, y=2, z=3, speed=0.5)

# Wait for the copter to arrive at the requested point
while True:
    telem = get_telemetry(frame_id=frame_id)
    # Calculating the distance to the requested point
    if get_distance(1, 2, 3, telem.x, telem.y, telem.z) < tolerance:
        # Arrived at the requested point
        break
    rospy.sleep(0.2)
```

### # {#disarm}

Quadcopter disarm (disabling propellers **the copter will fall down**):

```python
# Declaring a proxy:
from mavros_msgs.srv import CommandBool
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

# ...

arming(False)  # дизарм
```

### # {#transform}

Transforming the position (`PoseStamped`) from one system of coordinates ([of frame](frames.md)) to another one using [tf2] (http://wiki.ros.org/tf2):

```python
import tf2_ros
import tf2_geometry_msgs

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# ...

# Creating as PoseStamped object (or getting it from the topic):
pose = PoseStamped()
pose.header.frame_id = 'map' # frame, which is the position is specified
pose.header.stamp = rospy.get_rostime() # the moment for which the position is specified (current time)
pose.pose.position.x = 1
pose.pose.position.y = 2
pose.pose.position.z = 3
pose.pose.orientation.w = 1

frame_id = 'base_link' # target frame
transform_timeout = rospy.Duration(0.2) # wait for transformation timeout

# Transforming the position from the old frame to the new one:
new_pose = tf_buffer.transform(pose, frame_id, transform_timeout)
```

### # {#upside-down}

Determining whether the copter is turned over:

```python
PI_2 = math.pi / 2
telem = get_telemetry()

flipped = not -PI_2 <= telem.pitch <= PI_2 or not -PI_2 <= telem.roll <= PI_2
```

### # {#angle-hor}

Calculating the copter total angle to the horizon:

```python
PI_2 = math.pi / 2
telem = get_telemetry()

flipped = not -PI_2 <= telem.pitch <= PI_2 or not -PI_2 <= telem.roll <= PI_2
angle_to_horizon = math.atan(math.hypot(math.tan(telem.pitch), math.tan(telem.roll)))
if flipped:
    angle_to_horizon = math.pi - angle_to_horizon
```

### # {#circle}

Flying along a circular path:

```python
RADIUS = 0.6  # m
SPEED = 0.3 # rad / s

start = get_telemetry()
start_stamp = rospy.get_rostime()

r = rospy.Rate(10)

while not rospy.is_shutdown():
    angle = (rospy.get_rostime() - start_stamp).to_sec() * SPEED
    x = start.x + math.sin(angle) * RADIUS
    y = start.y + math.cos(angle) * RADIUS
    set_position(x=x, y=y, z=start.z)

    r.sleep()
```

### # {#rate}

repeating an action with the frequency of 10 Hz:

```python
r = rospy.Rate(10)
while not rospy.is_shutdown():
    # Do anything
    r.sleep()
```

### # {#mavros-sub}

An example of subscription to a topic from MAVROS

```python
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import RCIn

# ...

def state_update(pose):
    # Processing new data about the copter position
    pass

# Other handler functions
# ...

rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_update)
rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, velocity_update)
rospy.Subscriber('/mavros/battery', BatteryState, battery_update)
rospy.Subscriber('mavros/rc/in', RCIn, rc_callback)
```

Information about MAVROS topics us available at [the link](mavros.md).

<!-- markdownlint-disable MD044 -->

### # {#mavlink}

<!-- markdownlint-enable MD044 -->

An example of sending an arbitrary [MAVLink message](mavlink.md) to the copter:

```python
# ...

from mavros_msgs.msg import Mavlink
from mavros import mavlink
from pymavlink import mavutil

# ...

mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)

# Sending a HEARTBEAT message:

msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
ros_msg = mavlink.convert_to_rosmsg(msg)

mavlink_pub.publish(ros_msg)
```

### # {#rc-sub}

Reaction to switching the mode on the transmitter (may be used for starting an offline flight, see [example](https://gist.github.com/okalachev/b709f04522d2f9af97e835baedeb806b)):

```python
from mavros_msgs.msg import RCIn

# Called when new data are received from the transmitter
def rc_callback(data):
    # Arbitrary reaction to switching the toggle switch on the transmitter
        if data.channels[5] < 1100:
        # ...
        pass
    elif data.channels[5] > 1900:
        # ...
        pass
    else:
        # ...
        pass

# Creating a subscriber for the topic with the data from the transmitter
rospy.Subscriber('mavros/rc/in', RCIn, rc_callback)

rospy.spin()
```

### # {#flip}

Flip:

TODO