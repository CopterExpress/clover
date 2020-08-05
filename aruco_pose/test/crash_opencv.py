import rospy
import pytest

from visualization_msgs.msg import MarkerArray as VisMarkerArray


@pytest.fixture
def node():
    return rospy.init_node('aruco_pose_opencv_crash', anonymous=True)

def test_opencv_crashes_img01(node):
    rospy.wait_for_message('aruco_detect_01/visualization', VisMarkerArray, timeout=5)

def test_opencv_crashes_img02(node):
    rospy.wait_for_message('aruco_detect_02/visualization', VisMarkerArray, timeout=5)

def test_opencv_crashes_img03(node):
    rospy.wait_for_message('aruco_detect_03/visualization', VisMarkerArray, timeout=5)
