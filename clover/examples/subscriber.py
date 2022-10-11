# Information: https://clover.coex.tech/en/laser.html

import rospy
from sensor_msgs.msg import Range

rospy.init_node('process_rangefinder')

def range_callback(msg):
    # Process data from the rangefinder
    print('Rangefinder distance:', msg.range)

# Subscribe to laser rangefinder data
rospy.Subscriber('rangefinder/range', Range, range_callback)

rospy.spin()
