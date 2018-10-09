#!/usr/bin/env python
# TODO: rewrite, as Python version eats 20% CPU

from __future__ import division

import rospy
import VL53L1X
from sensor_msgs.msg import Range

rospy.init_node('vl53l1x')


# range_pub = rospy.Publisher('~range', Range, queue_size=5)
# TODO: why remmaping is not working?
range_pub = rospy.Publisher('mavros/distance_sensor/rangefinder_3_sub', Range, queue_size=10)
z_shift = rospy.get_param("z_shift", 0)  # TODO: move to mavros (use frame)

msg = Range()
msg.radiation_type = Range.INFRARED
msg.field_of_view = 0.471239
msg.min_range = 0
msg.max_range = 4
msg.header.frame_id = 'rangefinder'

tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open() # Initialise the i2c bus and configure the sensor
tof.start_ranging(3) # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range

rospy.loginfo('vl53l1x: start ranging')

r = rospy.Rate(14)
while not rospy.is_shutdown():
    msg.header.stamp = rospy.get_rostime()
    msg.range = tof.get_distance() / 1000 + z_shift
    range_pub.publish(msg)
    r.sleep()

tof.stop_ranging() # Stop ranging
