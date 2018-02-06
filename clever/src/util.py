from geometry_msgs.msg import Quaternion, Vector3, Point
import tf.transformations as t


def orientation_from_quaternion(q):
    return Quaternion(*q)


def orientation_from_euler(roll, pitch, yaw):
    q = t.quaternion_from_euler(roll, pitch, yaw)
    return orientation_from_quaternion(q)


def quaternion_from_orientation(o):
    return o.x, o.y, o.z, o.w


def euler_from_orientation(o):
    q = quaternion_from_orientation(o)
    return t.euler_from_quaternion(q)


def vector3_from_point(p):
    return Vector3(p.x, p.y, p.z)


def point_from_vector3(v):
    return Point(v.x, v.y, v.z)
