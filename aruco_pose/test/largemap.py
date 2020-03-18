import rospy
import pytest

from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray as VisMarkerArray

@pytest.fixture
def node():
    return rospy.init_node('test_aruco_largemap', anonymous=True)

def test_large_map_image(node):
    img = rospy.wait_for_message('aruco_map/image', Image, timeout=5)
    assert img.width == 2000
    assert img.height == 2000
    assert img.encoding in ('mono8', 'rgb8')

def test_large_map_visualization(node):
    vis = rospy.wait_for_message('aruco_map/visualization', VisMarkerArray, timeout=5)
    assert len(vis.markers) == 11
