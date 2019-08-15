import rospy
import pytest

from visualization_msgs.msg import MarkerArray as VisMarkerArray


@pytest.fixture
def node():
    return rospy.init_node('aruco_pose_test_empty_map', anonymous=True)

def test_empty_map(node):
    markers = rospy.wait_for_message('aruco_map/visualization', VisMarkerArray, timeout=5)
    assert len(markers.markers) == 0
