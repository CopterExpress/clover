#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf.transformations

from util import orientation_from_euler, euler_from_orientation


rospy.init_node('aruco_vpe')


LOOKUP_TIMEOUT = rospy.Duration(.1)
CAMERA_FRAME_ID = rospy.get_param('~camera_frame_id', 'bottom_camera_optical')


# TF2 stuff
tf_broadcaster = tf2_ros.TransformBroadcaster()
static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)


vision_position_pub = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=1)
_vision_position_pub = rospy.Publisher('fake_vision_pose', PoseStamped, queue_size=1)
last_published = None


q = Quaternion()
q.w = 1
ps = PoseStamped()
ps.pose.orientation = q


def send_transform(transform, child_frame_id):
    transform.child_frame_id = child_frame_id
    tf_broadcaster.sendTransform(transform)


vpe_posted = False


def publish_vpe(pose):
    stamp = pose.header.stamp

    global last_published, vpe_posted
    vpe_posted = True

    def lookup_transform(target_frame, source_frame):
        return tf_buffer.lookup_transform(target_frame, source_frame, stamp, LOOKUP_TIMEOUT)

    # Refine aruco_map
    reference_in_local_origin = lookup_transform('local_origin', 'aruco_map_reference')
    roll, pitch, yaw = euler_from_orientation(reference_in_local_origin.transform.rotation)
    reference_in_local_origin.transform.rotation = orientation_from_euler(0, 0, yaw)
    send_transform(reference_in_local_origin, 'aruco_map_reference_horiz')

    aruco_map_in_reference = lookup_transform('aruco_map_reference', 'aruco_map_raw')
    aruco_map_in_reference.header.frame_id = 'aruco_map_reference_horiz'
    send_transform(aruco_map_in_reference, 'aruco_map_vision')

    # Reset VPE
    if last_published is None or stamp - last_published > rospy.Duration(2):
        rospy.loginfo('Reset VPE')
        aruco_map_in_local_origin = lookup_transform('local_origin', 'aruco_map_vision')
        aruco_map_in_local_origin.child_frame_id = 'aruco_map'
        static_tf_broadcaster.sendTransform(aruco_map_in_local_origin)

    # Calculate VPE
    ps.header.frame_id = 'fcu_horiz'
    ps.header.stamp = stamp
    vpe_raw = tf_buffer.transform(ps, 'aruco_map_vision', LOOKUP_TIMEOUT)
    vpe_raw.header.frame_id = 'aruco_map'
    vpe = tf_buffer.transform(vpe_raw, 'local_origin', LOOKUP_TIMEOUT)
    _vision_position_pub.publish(vpe_raw)
    vision_position_pub.publish(vpe)
    last_published = stamp


rospy.Subscriber('aruco_pose/pose', PoseStamped, publish_vpe, queue_size=1)


local_pose = None


def handle_pose(data):
    global local_pose
    local_pose = data


rospy.Subscriber('mavros/local_position/pose', PoseStamped, handle_pose, queue_size=1)


rospy.loginfo('aruco_vpe inited')
r = rospy.Rate(5)


while not rospy.is_shutdown():
    if not vpe_posted:
        ps.header.stamp = rospy.get_rostime()
        vision_position_pub.publish(ps)

    r.sleep()
