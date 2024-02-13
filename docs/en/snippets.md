# Code examples

## Python

### # {#navigate_wait}

<a name="block-nav"></a><!-- old name of anchor -->

<a name="block-takeoff"></a><!-- old name of anchor -->

Function to fly to a point and wait for copter's arrival:

```python
import math

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
```

This function utilizes [`navigate_target`](frames.md#navigate_target) frame for computing the distance to the target.

Using the function for flying to the point x=3, y=2, z=1 in [marker's map](aruco_map.md):

```python
navigate_wait(x=3, y=2, z=1, frame_id='aruco_map')
```

This function can be used for taking off as well:

```python
navigate_wait(z=1, frame_id='body', auto_arm=True)
```

### # {#block-land}

<a name="block-land"></a><!-- old name of anchor -->

Land and wait until the copter lands:

```python
land()
while get_telemetry().armed:
    rospy.sleep(0.2)
```

Usage:

```python
land_wait()
```

### # {#wait_arrival}

Wait for copter's arrival to the [navigate](simple_offboard.md#navigate) target:

```python
import math

def wait_arrival(tolerance=0.2):
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
```

### # {#get_distance}

Calculate the distance between two points (**important**: the points are to be in the same [coordinate system](frames.md)):

```python
import math

def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
```

### # {#get_distance_global}

Approximation of distance (in meters) between two global coordinates (latitude/longitude):

```python
import math

def get_distance_global(lat1, lon1, lat2, lon2):
    return math.hypot(lat1 - lat2, lon1 - lon2) * 1.113195e5
```

### # {#disarm}

Disarm the drone (propellers will stop, **the drone will fall down**):

```python
# Declaring a proxy:
from mavros_msgs.srv import CommandBool
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

# ...

arming(False)  # disarm
```

### # {#transform}

Transform the position (`PoseStamped`) from one [coordinate system](frames.md) to another using [tf2](http://wiki.ros.org/tf2):

```python
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# ...

# Create PoseStamped object (or get it from a topic):
pose = PoseStamped()
pose.header.frame_id = 'map' # coordinate frame, in which the position is specified
pose.header.stamp = rospy.get_rostime() # the time for which the position is specified (current time)
pose.pose.position.x = 1
pose.pose.position.y = 2
pose.pose.position.z = 3
pose.pose.orientation.w = 1

frame_id = 'base_link' # target coordinate frame
transform_timeout = rospy.Duration(0.2) # timeout for transformation

# Transform the position from the old frame to the new one:
new_pose = tf_buffer.transform(pose, frame_id, transform_timeout)
```

### # {#upside-down}

Determine whether the copter is turned upside-down:

```python
PI_2 = math.pi / 2
telem = get_telemetry()

flipped = abs(telem.roll) > PI_2 or abs(telem.pitch) > PI_2
```

### # {#angle-hor}

Calculate the copter horizontal angle:

```python
PI_2 = math.pi / 2
telem = get_telemetry()

flipped = not -PI_2 <= telem.roll <= PI_2 or not -PI_2 <= telem.pitch <= PI_2
angle_to_horizon = math.atan(math.hypot(math.tan(telem.roll), math.tan(telem.pitch)))
if flipped:
    angle_to_horizon = math.pi - angle_to_horizon
```

### # {#circle}

Fly along a circular path:

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

Repeat an action at a frequency of 10 Hz:

```python
r = rospy.Rate(10)
while not rospy.is_shutdown():
    # Do anything
    r.sleep()
```

### # {#mavros-sub}

An example of subscription to a topic from MAVROS:

```python
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import RCIn

def pose_update(pose):
    # Processing new data of copter's position
    pass

rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_update)
rospy.Subscriber('mavros/local_position/velocity', TwistStamped, velocity_update)
rospy.Subscriber('mavros/battery', BatteryState, battery_update)
rospy.Subscriber('mavros/rc/in', RCIn, rc_callback)

rospy.spin()
```

Information about MAVROS topics is available at [the link](mavros.md).

<!-- markdownlint-disable MD044 -->

### # {#mavlink}

<!-- markdownlint-enable MD044 -->

Send an arbitrary [MAVLink message](mavlink.md) to the copter:

```python
from mavros_msgs.msg import Mavlink
from mavros import mavlink
from pymavlink import mavutil

mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)

# Sending a HEARTBEAT message:
msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
ros_msg = mavlink.convert_to_rosmsg(msg)

mavlink_pub.publish(ros_msg)
```

<!-- markdownlint-disable MD044 -->

### # {#mavlink-receive}

<!-- markdownlint-enable MD044 -->

Subscribe to all MAVLink messages from the flight controller and decode them:

```python
from mavros_msgs.msg import Mavlink
from mavros import mavlink
from pymavlink import mavutil

link = mavutil.mavlink.MAVLink('', 255, 1)

def mavlink_cb(msg):
    mav_msg = link.decode(mavlink.convert_to_bytes(msg))
    print('msgid =', msg.msgid, mav_msg) # print message id and parsed message

mavlink_sub = rospy.Subscriber('mavlink/from', Mavlink, mavlink_cb)

rospy.spin()
```

### # {#rc-sub}

React to the drone's mode switching (may be used for starting an autonomous flight, see [example](https://gist.github.com/okalachev/b709f04522d2f9af97e835baedeb806b)):

```python
from mavros_msgs.msg import RCIn

# Called when new data is received from the transmitter
def rc_callback(data):
    # React on toggling the mode of the transmitter
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

### # {#set_mode}

Change the [flight mode](modes.md) to arbitrary one:

```python
from mavros_msgs.srv import SetMode

set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

# ...

set_mode(custom_mode='STABILIZED')
```

### # {#flip}

Flip:

```python
import math

PI_2 = math.pi / 2

def flip():
    start = get_telemetry()  # memorize starting position

    set_rates(thrust=1)  # bump up
    rospy.sleep(0.2)

    set_rates(pitch_rate=30, thrust=0.2)  # pitch flip
    # set_rates(roll_rate=30, thrust=0.2)  # roll flip

    while True:
        telem = get_telemetry()
        flipped = abs(telem.roll) > PI_2 or abs(telem.pitch) > PI_2
        if flipped:
            break

    rospy.loginfo('finish flip')
    set_position(x=start.x, y=start.y, z=start.z, yaw=start.yaw)  # finish flip

print(navigate(z=2, speed=1, frame_id='body', auto_arm=True))  # take off
rospy.sleep(10)

rospy.loginfo('flip')
flip()
```

Requires the [special PX4 firmware for Clover](firmware.md#modified-firmware-for-clover). Before running a flip, take all necessary safety precautions.

### # {#calibrate-gyro}

Perform gyro calibration:

```python
from pymavlink import mavutil
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import State

send_command = rospy.ServiceProxy('mavros/cmd/command', CommandLong)

def calibrate_gyro():
    rospy.loginfo('Calibrate gyro')
    if not send_command(command=mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, param1=1).success:
        return False

    calibrating = False
    while not rospy.is_shutdown():
        state = rospy.wait_for_message('mavros/state', State)
        if state.system_status == mavutil.mavlink.MAV_STATE_CALIBRATING or state.system_status == mavutil.mavlink.MAV_STATE_UNINIT:
            calibrating = True
        elif calibrating and state.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
            rospy.loginfo('Calibrating finished')
            return True

calibrate_gyro()
```

> **Note** In process of calibration the drone should not be moved.

<!-- markdownlint-disable MD044 -->

### # {#aruco-detect-enabled}

<!-- markdownlint-enable MD044 -->

Enable and disable [ArUco markers recognition](aruco_marker.md) dynamically (for example, for saving CPU resources):

```python
import rospy
import dynamic_reconfigure.client

rospy.init_node('flight')
aruco_client = dynamic_reconfigure.client.Client('aruco_detect')

# Turn markers recognition off
aruco_client.update_configuration({'enabled': False})

rospy.sleep(5)

# Turn markers recognition on
aruco_client.update_configuration({'enabled': True})
```

### # {#optical-flow-enabled}

Enable and disable [Optical Flow](optical_flow.md) dynamically:

```python
import rospy
import dynamic_reconfigure.client

rospy.init_node('flight')
flow_client = dynamic_reconfigure.client.Client('optical_flow')

# Turn Optical Flow off
flow_client.update_configuration({'enabled': False})

rospy.sleep(5)

# Turn Optical Flow on
flow_client.update_configuration({'enabled': True})
```

<!-- markdownlint-disable MD044 -->

### # {#aruco-map-dynamic}

> **Info** For [RPi image](image.md) version > 0.23.

Change the used [ArUco markers map file](aruco_map.md) dynamically:

<!-- markdownlint-enable MD044 -->

```python
import rospy
import dynamic_reconfigure.client

rospy.init_node('flight')
map_client = dynamic_reconfigure.client.Client('aruco_map')

map_client.update_configuration({'map': '/home/pi/catkin_ws/src/clover/aruco_pose/map/office.txt'})
```

### # {#wait-global-position}

Wait for global position to appear (finishing [GPS receiver](gps.md) initialization):

```python
import math

while not rospy.is_shutdown():
    if math.isfinite(get_telemetry().lat):
        break
    rospy.sleep(0.2)
```

### # {#get-param}

Read flight controller's parameter:

```python
from mavros_msgs.srv import ParamGet
from mavros_msgs.msg import ParamValue

param_get = rospy.ServiceProxy('mavros/param/get', ParamGet)

# Read parameter of type INT
value = param_get(param_id='COM_FLTMODE1').value.integer

# Read parameter of type FLOAT
value = param_get(param_id='MPC_Z_P').value.float
```

### # {#set-param}

Set flight controller's parameter:

```python
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue

param_set = rospy.ServiceProxy('mavros/param/set', ParamSet)

# Set parameter of type INT:
param_set(param_id='COM_FLTMODE1', value=ParamValue(integer=8))

# Set parameter of type FLOAT:
param_set(param_id='MPC_Z_P', value=ParamValue(real=1.5))
```

### # {#is-simulation}

Check, if the code is running inside a [Gazebo simulation](simulation.md):

```python
is_simulation = rospy.get_param('/use_sim_time', False)
```

### # {#simulator-interaction}

You can move a physical object (link) in Gazebo (as well as change its velocity) using the `gazebo/set_link_state` service (of the type [`SetLinkState`](http://docs.ros.org/en/api/gazebo_msgs/html/srv/SetLinkState.html)). For example, if you add a cube to the world (link `unit_box::link`), you can move it to the point (1, 2, 3):

```python
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.msg import LinkState

rospy.init_node('flight')

set_link_state = rospy.ServiceProxy('gazebo/set_link_state', SetLinkState)

# Change link's position
set_link_state(LinkState(link_name='unit_box::link', pose=Pose(position=Point(1, 2, 3), orientation=Quaternion(0, 0, 0, 1))))
```

> **Info** Simple object animation in Gazebo can be implemented [using actors](http://classic.gazebosim.org/tutorials?tut=actor&cat=build_robot).
