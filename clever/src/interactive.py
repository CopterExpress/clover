#!/usr/bin/env python
import copy

import rospy
import tf.transformations as t
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from clever import srv


def make_box(msg):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.3
    marker.scale.y = msg.scale * 0.3
    marker.scale.z = msg.scale * 0.3
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    marker.pose.orientation.w = 1

    return marker


def make_box_control(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.orientation.w = 1
    control.markers.append(make_box(msg))
    msg.controls.append(control)
    return control


def make_quadcopter_marker():
    marker = InteractiveMarker()
    marker.header.frame_id = 'fcu'
    marker.header.stamp = rospy.get_rostime()
    marker.scale = 1
    marker.pose.orientation.w = 1

    marker.name = 'quadcopter'
    marker.description = 'Quadcopter'

    make_box_control(marker)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
    marker.controls.append(copy.deepcopy(control))
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    marker.controls.append(control)

    return marker


navigate = rospy.ServiceProxy('navigate', srv.Navigate)


def process_feedback(feedback):
    if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
        return

    p = feedback.pose.position
    o = feedback.pose.orientation
    yaw = t.euler_from_quaternion((o.x, o.y, o.z, o.w), axes='rzyx')[0]
    rospy.loginfo('Navigate to %s', p)
    rospy.loginfo(navigate(x=p.x, y=p.y, z=p.z, yaw=yaw, speed=2,
                           frame_id=feedback.header.frame_id, auto_arm=True))


rospy.init_node('quadcopter_im')

server = InteractiveMarkerServer('quadcopter_im')

int_marker = make_quadcopter_marker()
server.insert(int_marker, process_feedback)
server.applyChanges()

rospy.loginfo('Interactive quadcopter marker initialized')
rospy.spin()
