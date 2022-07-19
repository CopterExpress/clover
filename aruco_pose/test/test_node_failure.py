import rospy
import pytest

from visualization_msgs.msg import MarkerArray as VisMarkerArray
from aruco_pose.msg import MarkerArray


@pytest.fixture
def node():
    return rospy.init_node('aruco_pose_test', anonymous=True)

def test_node_failure(node):
    assert rospy.wait_for_message('aruco_map/visualization', VisMarkerArray, timeout=5).markers == []
    assert rospy.wait_for_message('aruco_map/map', MarkerArray, timeout=5).markers == []
