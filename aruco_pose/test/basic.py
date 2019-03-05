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


class TestArucoPose(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_aruco_detect', anonymous=True)

    def test_markers(self):
        markers = rospy.wait_for_message('aruco_detect/markers', MarkerArray, timeout=5)
        self.assertEqual(len(markers.markers), 4)
        self.assertEqual(markers.header.frame_id, 'main_camera_optical')

        self.assertEqual(markers.markers[0].id, 2)
        self.assertAlmostEqual(markers.markers[0].length, 0.33)
        self.assertAlmostEqual(markers.markers[0].pose.position.x, 0.36706567854)
        self.assertAlmostEqual(markers.markers[0].pose.position.y, 0.290484516644)
        self.assertAlmostEqual(markers.markers[0].pose.position.z, 2.18787602301)
        self.assertAlmostEqual(markers.markers[0].pose.orientation.x, 0.993997406299)
        self.assertAlmostEqual(markers.markers[0].pose.orientation.y, -0.00532003481626)
        self.assertAlmostEqual(markers.markers[0].pose.orientation.z, -0.107390951553)
        self.assertAlmostEqual(markers.markers[0].pose.orientation.w, 0.0201999263402)
        self.assertAlmostEqual(markers.markers[0].c1.x, 415.557739258)
        self.assertAlmostEqual(markers.markers[0].c1.y, 335.557739258)
        self.assertAlmostEqual(markers.markers[0].c2.x, 509.442260742)
        self.assertAlmostEqual(markers.markers[0].c2.y, 335.557739258)
        self.assertAlmostEqual(markers.markers[0].c3.x, 509.442260742)
        self.assertAlmostEqual(markers.markers[0].c3.y, 429.442260742)
        self.assertAlmostEqual(markers.markers[0].c4.x, 415.557739258)
        self.assertAlmostEqual(markers.markers[0].c4.y, 429.442260742)

        self.assertEqual(markers.markers[3].id, 3)
        self.assertAlmostEqual(markers.markers[3].length, 0.1)
        self.assertAlmostEqual(markers.markers[3].pose.position.x, -0.1805169666)
        self.assertAlmostEqual(markers.markers[3].pose.position.y, -0.200697302327)
        self.assertAlmostEqual(markers.markers[3].pose.position.z, 0.585767514823)
        self.assertAlmostEqual(markers.markers[3].pose.orientation.x, -0.961738074009)
        self.assertAlmostEqual(markers.markers[3].pose.orientation.y, -0.0375180244707)
        self.assertAlmostEqual(markers.markers[3].pose.orientation.z, -0.0115387773672)
        self.assertAlmostEqual(markers.markers[3].pose.orientation.w, 0.271144115664)
        self.assertAlmostEqual(markers.markers[3].c1.x, 129.557723999)
        self.assertAlmostEqual(markers.markers[3].c1.y, 49.557723999)
        self.assertAlmostEqual(markers.markers[3].c2.x, 223.442276001)
        self.assertAlmostEqual(markers.markers[3].c2.y, 49.557723999)
        self.assertAlmostEqual(markers.markers[3].c3.x, 223.442276001)
        self.assertAlmostEqual(markers.markers[3].c3.y, 143.442276001)
        self.assertAlmostEqual(markers.markers[3].c4.x, 129.557723999)
        self.assertAlmostEqual(markers.markers[3].c4.y, 143.442276001)

        self.assertEqual(markers.markers[1].id, 1)
        self.assertAlmostEqual(markers.markers[1].length, 0.33)
        self.assertEqual(markers.markers[2].id, 4)
        self.assertAlmostEqual(markers.markers[2].length, 0.33)

    def test_visualization(self):
        vis = rospy.wait_for_message('aruco_detect/visualization', VisMarkerArray, timeout=5)
        self.assertEqual(len(vis.markers), 9)

    def test_debug(self):
        img = rospy.wait_for_message('aruco_detect/debug', Image, timeout=5)
        self.assertEqual(img.width, 640)
        self.assertEqual(img.height, 480)
        self.assertEqual(img.header.frame_id, 'main_camera_optical')

    def test_map(self):
        pose = rospy.wait_for_message('aruco_map/pose', PoseWithCovarianceStamped, timeout=5)
        self.assertEqual(pose.header.frame_id, 'main_camera_optical')
        self.assertAlmostEqual(pose.pose.pose.position.x, -0.629167753342)
        self.assertAlmostEqual(pose.pose.pose.position.y, 0.293822650809)
        self.assertAlmostEqual(pose.pose.pose.position.z, 2.12641343155)
        self.assertAlmostEqual(pose.pose.pose.orientation.x, -0.998383794799)
        self.assertAlmostEqual(pose.pose.pose.orientation.y, -5.20919098575e-06)
        self.assertAlmostEqual(pose.pose.pose.orientation.z, -0.0300861070302)
        self.assertAlmostEqual(pose.pose.pose.orientation.w, 0.0482143590507)

    def test_map_image(self):
        img = rospy.wait_for_message('aruco_map/image', Image, timeout=5)
        self.assertEqual(img.width, 2000)
        self.assertEqual(img.height, 2000)
        self.assertEqual(img.encoding, 'mono8')

    def test_map_visualization(self):
        vis = rospy.wait_for_message('aruco_map/visualization', VisMarkerArray, timeout=5)

    def test_map_debug(self):
        img = rospy.wait_for_message('aruco_map/debug', Image, timeout=5)

    # def test_transforms(self):
    #     pass


rostest.rosrun('aruco_pose', 'test_aruco_detect', TestArucoPose, sys.argv)
