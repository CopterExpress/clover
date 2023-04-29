#!/usr/bin/env python3

import rospy
import math
import signal
import sys
import dynamic_reconfigure.client
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
from aruco_pose.msg import MarkerArray
from util import handle_response

rospy.init_node('autotest_aruco', disable_signals=True) # disable signals to allow interrupting with ctrl+c

try:
    flow_client = dynamic_reconfigure.client.Client('optical_flow', timeout=2)
except rospy.ROSException:
    flow_client = None
    print('Cannot configure optical flow, skip')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = handle_response(rospy.ServiceProxy('navigate', srv.Navigate))
land = handle_response(rospy.ServiceProxy('land', Trigger))

def interrupt(sig, frame):
    print('\nInterrupted, landing...')
    land()
    sys.exit(0)

signal.signal(signal.SIGINT, interrupt)

def print_current_map_position():
    telem = get_telemetry()
    dist = rospy.wait_for_message('rangefinder/range', Range).range
    print('Map position:\tx={:.1f}\ty={:.1f}\tz={:.1f}\tyaw={:.1f}\tdist={:.2f}'.format(telem.x, telem.y, telem.z, telem.yaw, dist))

def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)

markers = rospy.wait_for_message('aruco_map/map', MarkerArray, timeout=3)
left = min(marker.pose.position.x for marker in markers.markers)
bottom = min(marker.pose.position.y for marker in markers.markers)
width = max(marker.pose.position.x for marker in markers.markers)
height = max(marker.pose.position.y for marker in markers.markers)
center_x = left + width / 2
center_y = bottom + height / 2

print('Map rect: %g %g - %g %g' % (left, bottom, width, height))

input('Take off and hover 1 m [enter] ')
navigate_wait(x=0, y=0, z=1, frame_id='body', auto_arm=True)
print_current_map_position()

input('Go to corner %g %g 1.5 speed 1 [enter] ' % (width, height))
navigate_wait(x=width, y=height, z=1.5, speed=1, frame_id='aruco_map')
print_current_map_position()

input('Go to center %g %g 1.5 speed 5 [enter] ' % (center_x, center_y))
navigate_wait(x=center_x, y=center_y, z=1.5, speed=5, frame_id='aruco_map')
print_current_map_position()

if flow_client:
    input('Disable optical flow and keep hovering [enter] ')
    flow_client.update_configuration({'enabled': False})
    rospy.sleep(5)

    input('Enable optical flow back [enter] ')
    flow_client.update_configuration({'enabled': True})

input('Go to side 1 %g 2 heading top [enter] ' % (center_y))
navigate_wait(x=1, y=center_y, z=2, yaw=1.57, frame_id='aruco_map')
print_current_map_position()

marker_id = markers.markers[0].id
input('Go to marker %d z=1.5 [enter] ' % marker_id)
navigate_wait(x=0, y=0, z=1.5, yaw=0, frame_id='aruco_%d' % marker_id)
print_current_map_position()

input('Perform landing [enter] ')
land()
