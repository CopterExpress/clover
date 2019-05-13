#!/usr/bin/env python
import sys
import unittest
import json
import rospy
import rostest

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from aruco_pose.msg import MarkerArray
from visualization_msgs.msg import MarkerArray as VisMarkerArray


class TestArucoMapPass(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_parser_fail', anonymous=True)


    def test_node_failure(self):
        markers = rospy.wait_for_message('aruco_map/visualization', VisMarkerArray, timeout=5)
        self.assertEquals(len(markers.markers), 0)


rostest.rosrun('aruco_pose', 'test_aruco_map', TestArucoMapPass, sys.argv)
