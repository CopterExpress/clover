import rospy
import pytest

from visualization_msgs.msg import MarkerArray as VisMarkerArray


@pytest.fixture
def node():
    return rospy.init_node('aruco_pose_test', anonymous=True)

def test_node_failure(node):
    with pytest.raises(rospy.exceptions.ROSException):
        rospy.wait_for_message('aruco_map/visualization', VisMarkerArray, timeout=5)
