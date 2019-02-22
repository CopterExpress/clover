#!/usr/bin/env python
import sys
import unittest
import json
from pytest import approx
import rospy
import rostest

from sensor_msgs.msg import Image
from aruco_pose.msg import MarkerArray
from visualization_msgs.msg import MarkerArray as VisMarkerArray


def to_dict(msg):
    return {s: getattr(msg, s) for s in msg.__slots__}


class TestArucoDetect(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_aruco_detect', anonymous=True)

    def test_markers(self):
        markers = rospy.wait_for_message('/aruco_detect/markers', MarkerArray, timeout=5)
        assert len(markers.markers) == 4
        assert markers.header.frame_id == 'main_camera_optical'

        assert markers.markers[0].id == 2
        assert markers.markers[0].pose.pose.position.x == approx(0.36706567854)
        assert markers.markers[0].pose.pose.position.y == approx(0.290484516644)
        assert markers.markers[0].pose.pose.position.z == approx(2.18787602301)
        assert markers.markers[0].pose.pose.orientation.x == approx(0.993997406299)
        assert markers.markers[0].pose.pose.orientation.y == approx(-0.00532003481626)
        assert markers.markers[0].pose.pose.orientation.z == approx(-0.107390951553)
        assert markers.markers[0].pose.pose.orientation.w == approx(0.0201999263402)
        assert markers.markers[0].c1.x == approx(415.557739258)
        assert markers.markers[0].c1.y == approx(335.557739258)
        assert markers.markers[0].c2.x == approx(509.442260742)
        assert markers.markers[0].c2.y == approx(335.557739258)
        assert markers.markers[0].c3.x == approx(509.442260742)
        assert markers.markers[0].c3.y == approx(429.442260742)
        assert markers.markers[0].c4.x == approx(415.557739258)
        assert markers.markers[0].c4.y == approx(429.442260742)

        assert markers.markers[3].id == 3
        assert markers.markers[3].pose.pose.position.x == approx(-0.1805169666)
        assert markers.markers[3].pose.pose.position.y == approx(-0.200697302327)
        assert markers.markers[3].pose.pose.position.z == approx(0.585767514823)
        assert markers.markers[3].pose.pose.orientation.x == approx(-0.961738074009)
        assert markers.markers[3].pose.pose.orientation.y == approx(-0.0375180244707)
        assert markers.markers[3].pose.pose.orientation.z == approx(-0.0115387773672)
        assert markers.markers[3].pose.pose.orientation.w == approx(0.271144115664)
        assert markers.markers[3].c1.x == approx(129.557723999)
        assert markers.markers[3].c1.y == approx(49.557723999)
        assert markers.markers[3].c2.x == approx(223.442276001)
        assert markers.markers[3].c2.y == approx(49.557723999)
        assert markers.markers[3].c3.x == approx(223.442276001)
        assert markers.markers[3].c3.y == approx(143.442276001)
        assert markers.markers[3].c4.x == approx(129.557723999)
        assert markers.markers[3].c4.y == approx(143.442276001)

        assert markers.markers[1].id == 1
        assert markers.markers[2].id == 4

    def test_visualization(self):
        vis = rospy.wait_for_message('aruco_detect/visualization', VisMarkerArray, timeout=5)

    # def test_transforms(self):
    #     pass

rostest.rosrun('aruco_pose', 'test_aruco_detect', TestArucoDetect, sys.argv)
