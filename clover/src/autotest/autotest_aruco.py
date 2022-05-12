#!/usr/bin/env python3

import rospy
import math
import signal
import sys
from clover import srv
from threading import Thread
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range

rospy.init_node('autotest_aruco', disable_signals=True) # disable signals to allow interrupting with ctrl+c

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

def interrupt(sig, frame):
    print('\nInterrupted, landing...')
    land()
    sys.exit(0)

signal.signal(signal.SIGINT, interrupt)

def print_current_map_position():
    telem = get_telemetry()
    dist = rospy.wait_for_message('rangefinder/range', Range).range
    print('Map position:\tx={:.1f}\ty={:.1f}\tz={:.1f}\tyaw={:.1f}\tdist={:.2f}'.format(telem.x, telem.y, telem.z, telem.yaw, dist))

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), yaw_rate=0, speed=0.5, \
        frame_id='body', tolerance=0.2, auto_arm=False):

    res = navigate(x=x, y=y, z=z, yaw=yaw, yaw_rate=yaw_rate, speed=speed, \
        frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)

input('Take off and hover 1 m above the ground [enter] ')
navigate_wait(x=0, y=0, z=1, frame_id='body', auto_arm=True)
print_current_map_position()

input('Go to position\tx=1\ty=3\tz=2\tyaw=0 [enter] ')
navigate_wait(x=1, y=3, z=2, frame_id='aruco_map')
print_current_map_position()

input('Go to position\tx=0\ty=2\tz=1.2\tyaw=1.57\twith speed 5 [enter]')
navigate_wait(x=0, y=2, z=1.2, yaw=1.57, frame_id='aruco_map', speed=5)
print_current_map_position()

input('Go to marker 0, z=1.5 [enter] ')
navigate_wait(x=0, y=0, z=1.5, frame_id='aruco_0')
print_current_map_position()

input('Perform landing [enter] ')
land()
