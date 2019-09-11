#!/usr/bin/env python
import sys
import unittest
import json
import rospy
import rostest

from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray as VisMarkerArray


class TestArucoPose(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_aruco_largemap', anonymous=True)

    def test_map_image(self):
        img = rospy.wait_for_message('aruco_map/image', Image, timeout=5)
        self.assertEqual(img.width, 2000)
        self.assertEqual(img.height, 2000)
        self.assertIn(img.encoding, ('mono8', 'rgb8'))

    def test_map_visualization(self):
        vis = rospy.wait_for_message('aruco_map/visualization', VisMarkerArray, timeout=5)


rostest.rosrun('aruco_pose', 'test_aruco_largemap', TestArucoPose, sys.argv)
