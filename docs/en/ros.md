# ROS

<img src="../assets/ros.svg" width="200" align="right">

Main documentation: https://wiki.ros.org.

**ROS** is a widely used framework for developing complex and distributed robotic systems. The [Clover autonomous flights platform](programming.md) is based on ROS.

## Installation

ROS is already installed on [the RPi image](image.md).

To install ROS on your PC you may address the [official installation documentation](https://wiki.ros.org/noetic/Installation/Ubuntu). For a quick start it's recommended to use [the virtual machine image with ROS and Clover simulator](simulation_vm.md).

## Concepts

### Nodes

Main article: https://wiki.ros.org/Nodes.

ROS node is a special program (usually written in Python or C++) that communicates with other nodes via ROS topics and ROS services. Dividing complex robotic systems into isolated nodes provides certain advantages: reduced coupling of the code, increased reusability and reliability.

Many robotic libraries and drivers are made as ROS nodes.

In order to turn an ordinary program into a ROS node, include the `rospy` (Python) or `roscpp` (C++) library, and insert the initialization code.

An example of a ROS node in Python:

```python
import rospy

rospy.init_node('my_ros_node')  # the name of the ROS node

rospy.spin() # entering an infinite loop...
```

> **Info** Any [autonomous flight script](programming.md) for Clover is a ROS node.

### Topics

Main article: https://wiki.ros.org/Topics

A topic is a named data bus used by the nodes for exchanging messages. Any node can *publish* a message to any topic, and *subscribe* to any topic.

Для каждого созданного топика должен быть задан тип сообщений, которые по нему передаются. ROS включает в себя большое количество стандартных типов сообщений, покрывающих различные аспекты робототехники, но при необходимости возможно создание собственных типов сообщений. Примеры стандартных типов сообщений:

Each topic has the a of messages it passes. ROS include a lot of standard message types, covering different aspects of robotics. Creating custom message types is also possible. Example of standard message types:

|Message type|Description|
|-|-|
|[`std_msgs/Int64`](https://docs.ros.org/api/std_msgs/html/msg/Int64.html)|Integer number.|
|[`std_msgs/Float64`](https://docs.ros.org/api/std_msgs/html/msg/Float64.html)|Double-precision floating-point number.|
|[`std_msgs/String`](https://docs.ros.org/api/std_msgs/html/msg/String.html)|String.|
|[`geometry_msgs/PoseStamped`](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)|Position and orientation of an object in a given [coordinate system](frames.md) and a time stamp (widely used for passing the robot pose or some robot's part pose).|
|[`geometry_msgs/TwistStamped`](https://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html)|Linear and angular velocity of an object in a given coordinate system and a time stamp.|
|[`sensor_msgs/Image`](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)|Image (see the [article on working with the camera](camera.md)).|

> **Info** See the rest of standard message types in packages: [`common_msgs`](http://wiki.ros.org/common_msgs), [`std_msgs`](https://wiki.ros.org/std_msgs), [`geometry_msgs`](https://wiki.ros.org/geometry_msgs), [`sensor_msgs`](https://wiki.ros.org/sensor_msgs), and others.

Example of publishing a message of type [`String`]((https://docs.ros.org/api/std_msgs/html/msg/String.html)) in a topic `/foo` in Python:

```python
from std_msgs.msg import String

rospy.init_node('my_ros_node')

foo_pub = rospy.Publisher('/foo', String, queue_size=1)  # creating a Publisher

foo_pub.publish(data='Hello, world!')  # publishing the message
```

Example of subscription to a topic `/foo`:

```python
import rospy
from std_msgs.msg import String

rospy.init_node('my_ros_node')

def foo_callback(msg):
    print(msg.data)

# Subscribing. When a message is received in topic /foo, function foo_callback will be invoked.
rospy.Subscriber('/foo', String, foo_callback)
```

You can read a topic message once, using `wait_for_message` function:

```python
msg = rospy.wait_for_message('/foo', String, timeout=3)  # wait for a message in /foo topic with timeout of 3 seconds
```

You can also work with topics using the `rostopic` utility. For example, using the following command, you can view messages published in topic `/mavros/state`:

```bash
rostopic echo /mavros/state
```

The `rostopic info` command shows the type of messages in the topic, and `rostopic hz` shows frequency of published messages.

Also you can monitor and visualize topics using [graphical tools of ROS](rviz.md).

### Services

Main article: https://wiki.ros.org/Services

A service can be assimilated to the a function that can be called from one node, and processed in another one. The service has a name that is similar to the name of the topic, and 2 message types: request type and response type.

Thus, ROS services implement [remote procedure call (RPC)](https://en.wikipedia.org/wiki/Remote_procedure_call) pattern.

Example of invoking a ROS service in Python:

```python
import rospy
from clover.srv import GetTelemetry

rospy.init_node('my_ros_node')

# Creating a wrapper for the get_telemetry service of the clover package with the GetTelemetry type:
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

# Invoking the service, and getting the quadcopter telemetry:
telemetry = get_telemetry()
```

You can also work with the services using the `rosservice` utility. For instance, you can call service `/get_telemetry` from the command line:

```bash
rosservice call /get_telemetry "{frame_id: ''}"
```

More examples of using the services for Clover quadcopter autonomous flights are available in the [documentation for node simple_offboard](simple_offboard.md).

### Names

Main article: https://wiki.ros.org/Names.

Any topic, service or a parameter is identified with a unique name. A ROS name is hierarchical structure with a `/` symbol as a separator (which is close to a file name in a file system).

Examples of ROS names:

* `/` (global namespace)
* `/foo`
* `/stanford/robot/name`
* `/wg/node1`

This names are global (close to global names in a file system). In practice, it's recommended to use *private* or *relative* names.

#### Private name

Each node can use its own private namespace (corresponding its name) for its resources. For example, `aruco_detect` node may publish such topics:

* `/aruco_detect/markers`
* `/aruco_detect/visualization`
* `/aruco_detect/debug`

When a node is referring its private resource, instead of `/aruco_detect/` namespace it may use `~` symbol:

* `~markers`
* `~visualization`
* `~debug`

Thus, creating a `foo` topic and the private namespace would look like this:

```python
private_foo_pub = rospy.Publisher('~foo', String, queue_size=1)
```

#### Relative name

Several nodes may group into a common namespace (for example, when there are several robots in the network). For referring topics and services in the current namespace, the opening `/` symbol is omitted.

Example of create a `foo` topic in the current namespace:

```python
relative_foo_pub = rospy.Publisher('foo', String, queue_size=1)
```

> **Hint** Generally, it's recommended to use private or relative names instead of global ones.

### Working on several PCs

Main article: http://wiki.ros.org/ROS/Tutorials/MultipleMachines.

The advantage of using ROS is the possibility of distributing the nodes across several PCs in the network. For example, a node that recognizes an image may be run on a more powerful PC; the node that controls the copter may be run directly on a Raspberry Pi connected to the flight controller, etc.
