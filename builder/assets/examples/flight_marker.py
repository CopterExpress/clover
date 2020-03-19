# Information: https://clever.coex.tech/en/programming.html

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

# Take off and hover 1 m above the ground
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)

# Wait for 3 seconds
rospy.sleep(3)

# Fly 1 meter above ArUco marker 0
navigate(x=0, y=0, z=1, frame_id='aruco_0')

# Wait for 3 seconds
rospy.sleep(3)

# Fly to x=1 y=1 z=1 relative to ArUco markers map
navigate(x=1, y=1, z=1, frame_id='aruco_map')

# Wait for 3 seconds
rospy.sleep(3)

# Perform landing
land()
