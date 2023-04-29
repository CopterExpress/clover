#!/usr/bin/env python3

import rospy
import math
from math import nan, inf
import signal
import sys
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
from util import handle_response

rospy.init_node('autotest_flight', disable_signals=True) # disable signals to allow interrupting with ctrl+c

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = handle_response(rospy.ServiceProxy('navigate', srv.Navigate))
navigate_global = handle_response(rospy.ServiceProxy('navigate_global', srv.NavigateGlobal))
set_yaw = handle_response(rospy.ServiceProxy('set_yaw', srv.SetYaw))
set_yaw_rate = handle_response(rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate))
set_position = handle_response(rospy.ServiceProxy('set_position', srv.SetPosition))
set_velocity = handle_response(rospy.ServiceProxy('set_velocity', srv.SetVelocity))
set_attitude = handle_response(rospy.ServiceProxy('set_attitude', srv.SetAttitude))
set_rates = handle_response(rospy.ServiceProxy('set_rates', srv.SetRates))
land = handle_response(rospy.ServiceProxy('land', Trigger))

def interrupt(sig, frame):
    print('\nInterrupted, landing...')
    land()
    sys.exit(0)

signal.signal(signal.SIGINT, interrupt)

def navigate_wait(x=0, y=0, z=0, yaw=nan, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)

def print_distance():
    dist = rospy.wait_for_message('rangefinder/range', Range).range
    print('Distance: {:.2f}'.format(dist))

input('Take off and hover 1 m [enter] ')
navigate_wait(z=1, frame_id='body', auto_arm=True)
print_distance()
start = get_telemetry()

input('Fly forward 2 m [enter] ')
navigate_wait(x=2, frame_id='navigate_target')
print_distance()

input('Climb 0.5 m [enter] ')
navigate_wait(z=0.5, frame_id='navigate_target')
print_distance()

input('Rotate left 90° [enter] ')
navigate(yaw=math.pi / 2, frame_id='navigate_target')
rospy.sleep(3)

input('Use set_velocity to fly forward 2 m speed 1 [enter]')
set_velocity(vx=1, vy=0.0, vz=0, frame_id='body')
rospy.sleep(2)
set_position(frame_id='body')

input('Rotate right 90° using set_yaw [enter] ')
set_yaw(yaw=-math.pi / 2, frame_id='navigate_target')
rospy.sleep(3)

input('Use set_attitude to fly backwards [enter]')
set_attitude(roll=0, pitch=-0.3, yaw=0, thrust=0.5, frame_id='body')
rospy.sleep(0.3)
set_position(frame_id='body')

input('Use set_attitude to fly right [enter]')
set_attitude(roll=0.3, pitch=0, yaw=0, thrust=0.5, frame_id='body')
rospy.sleep(0.5)
set_position(frame_id='body')

input('Use set_rates to fly right [enter]')
set_rates(roll_rate=1.2, thrust=0.5)
rospy.sleep(0.4)
set_position(frame_id='body')

input('Rotate 360° to the right using set_yaw_rate [enter]')
set_yaw_rate(yaw_rate=-1)
rospy.sleep(2 * math.pi)
set_position(frame_id='body')

input('Return to start point heading forward [enter]')
navigate_wait(x=start.x, y=start.y, z=start.z, yaw=inf, speed=1, frame_id='map')

input('Land [enter]')
land()
