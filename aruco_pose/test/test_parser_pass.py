import rospy
import pytest

from sensor_msgs.msg import Image
from aruco_pose.msg import MarkerArray
from visualization_msgs.msg import MarkerArray as VisMarkerArray


@pytest.fixture
def node():
    return rospy.init_node('aruco_pose_test', anonymous=True)

def approx(expected):
    return pytest.approx(expected, abs=1e-4) # compare floats more roughly

def test_markers(node):
    markers = rospy.wait_for_message('aruco_map/visualization', VisMarkerArray, timeout=5)
    assert len(markers.markers) == 6

    assert markers.markers[0].pose.position.x == approx(0)
    assert markers.markers[0].pose.position.y == approx(0)
    assert markers.markers[0].pose.position.z == approx(0)

    assert markers.markers[1].pose.position.x == approx(1)
    assert markers.markers[1].pose.position.y == approx(1)
    assert markers.markers[1].pose.position.z == approx(1)

    assert markers.markers[2].pose.position.x == approx(1)
    assert markers.markers[2].pose.position.y == approx(0)
    assert markers.markers[2].pose.position.z == approx(0.5)

    assert markers.markers[3].pose.position.x == approx(0)
    assert markers.markers[3].pose.position.y == approx(1)
    assert markers.markers[3].pose.position.z == approx(0)

    assert markers.markers[4].pose.position.x == approx(1)
    assert markers.markers[4].pose.position.y == approx(0.5)
    assert markers.markers[4].pose.position.z == approx(0)

    assert markers.markers[5].pose.position.x == approx(2.2)
    assert markers.markers[5].pose.position.y == approx(0.2)
    assert markers.markers[5].pose.position.z == approx(0)

    assert markers.markers[0].scale.x == approx(0.33)
    assert markers.markers[0].scale.y == approx(0.33)
    assert markers.markers[1].scale.x == approx(0.225)
    assert markers.markers[1].scale.y == approx(0.225)
    assert markers.markers[2].scale.x == approx(0.45)
    assert markers.markers[2].scale.y == approx(0.45)
    assert markers.markers[3].scale.x == approx(0.15)
    assert markers.markers[3].scale.y == approx(0.15)
    assert markers.markers[4].scale.x == approx(0.25)
    assert markers.markers[4].scale.y == approx(0.25)
    assert markers.markers[5].scale.x == approx(0.35)
    assert markers.markers[5].scale.y == approx(0.35)

def test_map_image(node):
    img = rospy.wait_for_message('aruco_map/image', Image, timeout=5)
    assert img.width == 2000
    assert img.height == 2000
    assert img.encoding in ('mono8', 'rgb8')
