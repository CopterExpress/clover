#!/usr/bin/env python
from __future__ import division

import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Vector3, Vector3Stamped
import tf2_ros
import tf2_geometry_msgs
from mavros_msgs.msg import PositionTarget, AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from threading import Lock
import math

from global_local import global_to_local
from util import euler_from_orientation, vector3_from_point, orientation_from_euler
from std_srvs.srv import Trigger
from clever import srv


rospy.init_node('simple_offboard')


# TF2 stuff
tf_broadcaster = tf2_ros.TransformBroadcaster()
static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)


def init_fcu_horiz():
    # `fcu_horiz` frame publishing

    tr = TransformStamped()
    tr.header.frame_id = 'local_origin'
    tr.child_frame_id = 'fcu_horiz'

    def update_pose(data):
        tr.header.stamp = data.header.stamp
        tr.transform.translation = vector3_from_point(data.pose.position)
        yaw = euler_from_orientation(data.pose.orientation)[2]
        tr.transform.rotation = orientation_from_euler(0, 0, yaw)
        tf_broadcaster.sendTransform(tr)

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, update_pose)


init_fcu_horiz()


position_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)


state = None


def state_update(data):
    global state
    state = data


rospy.Subscriber('/mavros/state', State, state_update)


AUTO_OFFBOARD = rospy.get_param('~auto_offboard', True)
AUTO_ARM = AUTO_OFFBOARD and rospy.get_param('~auto_arm', True)
OFFBOARD_TIMEOUT = rospy.Duration(rospy.get_param('~offboard_timeout', 3))
ARM_TIMEOUT = rospy.Duration(rospy.get_param('~arm_timeout', 5))
TRANSFORM_TIMEOUT = rospy.Duration(rospy.get_param('~transform_timeout', 3))
SETPOINT_RATE = rospy.get_param('~setpoint_rate', 50)


def offboard_and_arm():
    if AUTO_OFFBOARD and state.mode != 'OFFBOARD':
        rospy.sleep(.3)
        rospy.loginfo('Switch mode to OFFBOARD')
        res = set_mode(base_mode=0, custom_mode='OFFBOARD')

        start = rospy.get_rostime()
        while True:
            if state.mode == 'OFFBOARD':
                break
            if rospy.get_rostime() - start > OFFBOARD_TIMEOUT:
                raise Exception('OFFBOARD request timed out')

    if AUTO_ARM and not state.armed:
        rospy.loginfo('Arming')
        res = arming(True)

        start = rospy.get_rostime()
        while True:
            if state.armed:
                return True

            if rospy.get_rostime() - start > ARM_TIMEOUT:
                raise Exception('Arming timed out')


ps = PoseStamped()
vs = Vector3Stamped()


def get_publisher_and_message(req, stamp):
    ps.header.stamp = stamp
    vs.header.stamp = stamp

    if isinstance(req, srv.SetPositionRequest):
        ps.header.frame_id = req.frame_id or 'local_origin'
        ps.pose.position = Point(req.x, req.y, req.z)
        ps.pose.orientation = orientation_from_euler(0, 0, req.yaw)
        pose_local = tf_buffer.transform(ps, 'local_origin', TRANSFORM_TIMEOUT)

        msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                             type_mask=PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                       PositionTarget.IGNORE_YAW_RATE,
                             position=pose_local.pose.position,
                             yaw=euler_from_orientation(pose_local.pose.orientation)[2] - math.pi / 2)
        return position_pub, msg

    elif isinstance(req, srv.SetPositionYawRateRequest):
        ps.header.frame_id = req.frame_id or 'local_origin'
        ps.pose.position = Point(req.x, req.y, req.z)
        pose_local = tf_buffer.transform(ps, 'local_origin', TRANSFORM_TIMEOUT)
        msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                             type_mask=PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                       PositionTarget.IGNORE_YAW,
                             position=pose_local.pose.position,
                             yaw_rate=req.yaw_rate)
        return position_pub, msg

    elif isinstance(req, srv.SetPositionGlobalRequest):
        x, y = global_to_local(req.lat, req.lon)

        ps.header.frame_id = req.frame_id or 'local_origin'
        ps.pose.position = Point(0, 0, req.z)
        ps.pose.orientation = orientation_from_euler(0, 0, req.yaw)
        pose_local = tf_buffer.transform(ps, 'local_origin', TRANSFORM_TIMEOUT)
        pose_local.pose.position.x = x
        pose_local.pose.position.y = y

        msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                             type_mask=PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                       PositionTarget.IGNORE_YAW_RATE,
                             position=pose_local.pose.position,
                             yaw=euler_from_orientation(pose_local.pose.orientation)[2] - math.pi / 2)
        return position_pub, msg

    elif isinstance(req, srv.SetPositionGlobalYawRateRequest):
        x, y = global_to_local(req.lat, req.lon)

        ps.header.frame_id = req.frame_id or 'local_origin'
        ps.pose.position = Point(0, 0, req.z)
        pose_local = tf_buffer.transform(ps, 'local_origin', TRANSFORM_TIMEOUT)
        pose_local.pose.position.x = x
        pose_local.pose.position.y = y

        msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                             type_mask=PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                       PositionTarget.IGNORE_YAW,
                             position=pose_local.pose.position,
                             yaw_rate=req.yaw_rate)
        return position_pub, msg

    elif isinstance(req, srv.SetVelocityRequest):
        vs.vector = Vector3(req.vx, req.vy, req.vz)
        vs.header.frame_id = req.frame_id or 'local_origin'
        ps.header.frame_id = req.frame_id or 'local_origin'
        ps.pose.orientation = orientation_from_euler(0, 0, req.yaw)
        pose_local = tf_buffer.transform(ps, 'local_origin', TRANSFORM_TIMEOUT)
        vector_local = tf_buffer.transform(vs, 'local_origin', TRANSFORM_TIMEOUT)
        msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                             type_mask=PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                       PositionTarget.IGNORE_YAW_RATE,
                             velocity=vector_local.vector,
                             yaw=euler_from_orientation(pose_local.pose.orientation)[2] - math.pi / 2)
        return position_pub, msg

    elif isinstance(req, srv.SetVelocityYawRateRequest):
        vs.vector = Vector3(req.vx, req.vy, req.vz)
        vs.header.frame_id = req.frame_id or 'local_origin'
        vector_local = tf_buffer.transform(vs, 'local_origin', TRANSFORM_TIMEOUT)
        msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                             type_mask=PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                       PositionTarget.IGNORE_YAW,
                             velocity=vector_local.vector,
                             yaw_rate=req.yaw_rate)
        return position_pub, msg

    elif isinstance(req, srv.SetAttitudeRequest):
        ps.header.frame_id = req.frame_id or 'local_origin'
        ps.pose.orientation = orientation_from_euler(req.roll, req.pitch, req.yaw)
        pose_local = tf_buffer.transform(ps, 'local_origin', TRANSFORM_TIMEOUT)
        msg = AttitudeTarget(orientation=pose_local.pose.orientation,
                             thrust=req.thrust,
                             type_mask=AttitudeTarget.IGNORE_YAW_RATE + AttitudeTarget.IGNORE_PITCH_RATE +
                                       AttitudeTarget.IGNORE_ROLL_RATE)
        return attitude_pub, msg

    elif isinstance(req, srv.SetAttitudeYawRateRequest):
        msg = AttitudeTarget(orientation=orientation_from_euler(req.roll, req.pitch, 0),
                             thrust=req.thrust,
                             type_mask=AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_ROLL_RATE)
        msg.body_rate.z = req.yaw_rate
        return attitude_pub, msg

    elif isinstance(req, srv.SetRatesYawRequest):
        ps.header.frame_id = req.frame_id or 'local_origin'
        ps.pose.orientation = orientation_from_euler(0, 0, req.yaw)
        pose_local = tf_buffer.transform(ps, 'local_origin', TRANSFORM_TIMEOUT)
        msg = AttitudeTarget(orientation=pose_local.pose.orientation,
                             thrust=req.thrust,
                             type_mask=AttitudeTarget.IGNORE_YAW_RATE,
                             body_rate=Vector3(req.roll_rate, req.pitch_rate, 0))
        return attitude_pub, msg

    elif isinstance(req, srv.SetRatesRequest):
        msg = AttitudeTarget(thrust=req.thrust,
                             type_mask=AttitudeTarget.IGNORE_ATTITUDE,
                             body_rate=Vector3(req.roll_rate, req.pitch_rate, req.yaw_rate))
        return attitude_pub, msg


current_pub = None
current_msg = None
current_req = None
handle_lock = Lock()


def handle(req):
    global current_pub, current_msg, current_req
    with handle_lock:
        try:
            stamp = rospy.get_rostime()
            current_pub, current_msg = get_publisher_and_message(req, stamp)
            rospy.loginfo('Topic: %s, message: %s', current_pub.name, current_msg)

            current_msg.header.stamp = stamp
            current_pub.publish(current_msg)

            offboard_and_arm()
            return {'success': True}

        except Exception as e:
            rospy.logerr(str(e))
            return {'success': False, 'message': str(e)}


def release(req):
    global current_pub
    current_pub = None
    rospy.loginfo('simple_offboard: release')
    return {'success': True}


rospy.Service('set_position', srv.SetPosition, handle)
rospy.Service('set_position/yaw_rate', srv.SetPositionYawRate, handle)
rospy.Service('set_position_global', srv.SetPositionGlobal, handle)
rospy.Service('set_position_global/yaw_rate', srv.SetPositionGlobalYawRate, handle)
rospy.Service('set_velocity', srv.SetVelocity, handle)
rospy.Service('set_velocity/yaw_rate', srv.SetVelocityYawRate, handle)
rospy.Service('set_attitude', srv.SetAttitude, handle)
rospy.Service('set_attitude/yaw_rate', srv.SetAttitudeYawRate, handle)
rospy.Service('set_rates', srv.SetRates, handle)
rospy.Service('set_rates/yaw', srv.SetRatesYaw, handle)
rospy.Service('release', Trigger, release)


rospy.loginfo('simple_offboard inited')


def start_loop():
    global current_pub, current_msg, current_req
    r = rospy.Rate(SETPOINT_RATE)

    while not rospy.is_shutdown():
        with handle_lock:
            if current_pub is not None:
                try:
                    stamp = rospy.get_rostime()

                    if getattr(current_req, 'update_frame', False):
                        current_pub, current_msg = get_publisher_and_message(current_req, stamp)

                    current_msg.header.stamp = stamp
                    current_pub.publish(current_msg)

                except Exception as e:
                    rospy.logwarn_throttle(10, str(e))

        r.sleep()


start_loop()
