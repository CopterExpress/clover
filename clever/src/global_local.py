import rospy
import math
import geopy
from geometry_msgs.msg import PoseStamped
from geopy.distance import VincentyDistance, vincenty
from sensor_msgs.msg import NavSatFix


def global_to_local(lat, lon):
    # TODO: refactor

    position_global = rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=5)
    pose = rospy.wait_for_message('mavros/local_position/pose', PoseStamped, timeout=5)

    d = math.hypot(pose.pose.position.x, pose.pose.position.y)

    bearing = math.degrees(math.atan2(-pose.pose.position.x, -pose.pose.position.y))
    if bearing < 0:
        bearing += 360

    cur = geopy.Point(position_global.latitude, position_global.longitude)
    origin = VincentyDistance(meters=d).destination(cur, bearing)

    _origin = origin.latitude, origin.longitude
    olat_tlon = origin.latitude, lon
    tlat_olon = lat, origin.longitude

    N = vincenty(_origin, tlat_olon)
    if lat < origin.latitude:
        N = -N

    E = vincenty(_origin, olat_tlon)
    if lon < origin.longitude:
        E = -E

    return E.meters, N.meters
