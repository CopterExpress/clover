#!/usr/bin/env python
import sys
import unittest
import json
from pytest import approx
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
        assert len(markers.markers) == 4
        assert markers.header.frame_id == 'main_camera_optical'

        assert markers.markers[0].id == 2
        assert markers.markers[0].length == approx(0.33)
        assert markers.markers[0].pose.position.x == approx(0.36706567854)
        assert markers.markers[0].pose.position.y == approx(0.290484516644)
        assert markers.markers[0].pose.position.z == approx(2.18787602301)
        assert markers.markers[0].pose.orientation.x == approx(0.993997406299)
        assert markers.markers[0].pose.orientation.y == approx(-0.00532003481626)
        assert markers.markers[0].pose.orientation.z == approx(-0.107390951553)
        assert markers.markers[0].pose.orientation.w == approx(0.0201999263402)
        assert markers.markers[0].c1.x == approx(415.557739258)
        assert markers.markers[0].c1.y == approx(335.557739258)
        assert markers.markers[0].c2.x == approx(509.442260742)
        assert markers.markers[0].c2.y == approx(335.557739258)
        assert markers.markers[0].c3.x == approx(509.442260742)
        assert markers.markers[0].c3.y == approx(429.442260742)
        assert markers.markers[0].c4.x == approx(415.557739258)
        assert markers.markers[0].c4.y == approx(429.442260742)

        assert markers.markers[3].id == 3
        assert markers.markers[3].length == approx(0.1)
        assert markers.markers[3].pose.position.x == approx(-0.1805169666)
        assert markers.markers[3].pose.position.y == approx(-0.200697302327)
        assert markers.markers[3].pose.position.z == approx(0.585767514823)
        assert markers.markers[3].pose.orientation.x == approx(-0.961738074009)
        assert markers.markers[3].pose.orientation.y == approx(-0.0375180244707)
        assert markers.markers[3].pose.orientation.z == approx(-0.0115387773672)
        assert markers.markers[3].pose.orientation.w == approx(0.271144115664)
        assert markers.markers[3].c1.x == approx(129.557723999)
        assert markers.markers[3].c1.y == approx(49.557723999)
        assert markers.markers[3].c2.x == approx(223.442276001)
        assert markers.markers[3].c2.y == approx(49.557723999)
        assert markers.markers[3].c3.x == approx(223.442276001)
        assert markers.markers[3].c3.y == approx(143.442276001)
        assert markers.markers[3].c4.x == approx(129.557723999)
        assert markers.markers[3].c4.y == approx(143.442276001)

        assert markers.markers[1].id == 1
        assert markers.markers[1].length == approx(0.33)
        assert markers.markers[2].id == 4
        assert markers.markers[2].length == approx(0.33)

    def test_visualization(self):
        vis = rospy.wait_for_message('aruco_detect/visualization', VisMarkerArray, timeout=5)

    def test_debug(self):
        img = rospy.wait_for_message('aruco_detect/debug', Image, timeout=5)

    def test_map(self):
        pose = rospy.wait_for_message('aruco_map/pose', PoseWithCovarianceStamped, timeout=5)
        assert pose.header.frame_id == 'main_camera_optical'
        assert pose.pose.pose.position.x == approx(-0.629167753342)
        assert pose.pose.pose.position.y == approx(0.293822650809)
        assert pose.pose.pose.position.z == approx(2.12641343155)
        assert pose.pose.pose.orientation.x == approx(-0.998383794799)
        assert pose.pose.pose.orientation.y == approx(-5.20919098575e-06)
        assert pose.pose.pose.orientation.z == approx(-0.0300861070302)
        assert pose.pose.pose.orientation.w == approx(0.0482143590507)

    def test_map_image(self):
        img = rospy.wait_for_message('aruco_map/image', Image, timeout=5)

    def test_map_visualization(self):
        vis = rospy.wait_for_message('aruco_map/visualization', VisMarkerArray, timeout=5)

    def test_map_debug(self):
        img = rospy.wait_for_message('aruco_map/debug', Image, timeout=5)

    # def test_transforms(self):
    #     pass


rostest.rosrun('aruco_pose', 'test_aruco_detect', TestArucoPose, sys.argv)
