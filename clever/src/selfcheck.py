#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, Imu
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped


# TODO: roscore is running
# TODO: local_origin, fcu, fcu_horiz
# TODO: rc service
# TODO: perform commander check in PX4


rospy.init_node('selfcheck')


def check_fcu():
    try:
        state = rospy.wait_for_message('mavros/state', State, timeout=3)
        if not state.connected:
            raise Exception('No connection to the FCU')
    except rospy.ROSException:
        raise Exception('No MAVROS state')


def check_camera(name):
    try:
        rospy.wait_for_message(name + '/image_raw', Image, timeout=1)
    except rospy.ROSException:
        raise Exception('No %s camera images' % name)
    try:
        rospy.wait_for_message(name + '/camera_info', CameraInfo, timeout=3)
    except rospy.ROSException:
        raise Exception('No %s camera camera info' % name)

def check_aruco():
    try:
        rospy.wait_for_message('aruco_pose/debug', Image, timeout=1)
    except rospy.ROSException:
        raise Exception('No aruco_pose/debug topic')


def check_simpleoffboard():
    try:
        rospy.wait_for_service('navigate', timeout=3)
        rospy.wait_for_service('get_telemetry', timeout=3)
        rospy.wait_for_service('land', timeout=3)
    except rospy.ROSException:
        raise Exception('No simple_offboard services')


def check_imu():
    try:
        rospy.wait_for_message('mavros/imu/data', Imu, timeout=1)
    except rospy.ROSException:
        raise Exception('No IMU data')


def check_local_position():
    try:
        rospy.wait_for_message('mavros/local_position/pose', PoseStamped, timeout=1)
    except rospy.ROSException:
        raise Exception('No local position')


def check_velocity():
    try:
        velocity = rospy.wait_for_message('mavros/local_position/velocity', TwistStamped, timeout=1)
        horiz = math.hypot(velocity.twist.linear.x, velocity.twist.linear.y)
        vert = velocity.twist.linear.z
        if abs(horiz) > 0.1:
            raise Exception('Horizontal velocity estimation is %s m/s; is the copter staying still?' % horiz)
        if abs(vert) > 0.1:
            raise Exception('Vertical velocity estimation is %s m/s; is the copter staying still?' % vert)
    except rospy.ROSException:
        raise Exception('No velocity estimation')


def check_global_position():
    try:
        rospy.wait_for_message('mavros/global_position/global', PoseStamped, timeout=3)
    except rospy.ROSException:
        raise Exception('No global position')


def check(name, fn):
    try:
        fn()
        rospy.loginfo('%s: OK', name)
    except Exception as e:
        rospy.logwarn('%s: %s', name, str(e))


def selfcheck():
    check('FCU', check_fcu)
    check('Simple offboard node', check_simpleoffboard)
    check('Main camera node', lambda: check_camera('main_camera'))
    check('aruco_pose/debug topic', check_aruco)
    check('IMU data', check_imu)
    check('Local position', check_local_position)
    check('Velocity estimation', check_velocity)
    check('Global position (GPS)', check_global_position)


if __name__ == '__main__':
    rospy.loginfo('Performing selfcheck...')
    selfcheck()
