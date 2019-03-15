#!/usr/bin/env python

import math
from subprocess import Popen, PIPE
import re
import traceback
import rospy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, Imu, Range
from mavros_msgs.msg import State, OpticalFlowRad
from geometry_msgs.msg import PoseStamped, TwistStamped
import tf.transformations as t


# TODO: roscore is running
# TODO: clever.service is running
# TODO: check attitude is present
# TODO: disk free space
# TODO: map, base_link, body
# TODO: rc service
# TODO: perform commander check, ekf2 status on PX4
# TODO: check if FCU params setter succeed
# TODO: selfcheck ROS service (with blacklists for checks)


rospy.init_node('selfcheck')


failures = []


def failure(text, *args):
    failures.append(text % args)


def check(name):
    def inner(fn):
        def wrapper(*args, **kwargs):
            failures[:] = []
            try:
                fn(*args, **kwargs)
                for f in failures:
                    rospy.logwarn('%s: %s', name, f)
            except Exception as e:
                traceback.print_exc()
                rospy.logwarn('%s: exception occurred', name)
                return
            if not failures:
                rospy.loginfo('%s: OK', name)
        return wrapper
    return inner


@check('FCU')
def check_fcu():
    try:
        state = rospy.wait_for_message('mavros/state', State, timeout=3)
        if not state.connected:
            failure('no connection to the FCU (check wiring)')
    except rospy.ROSException:
        failure('no MAVROS state (check wiring)')


@check('Camera')
def check_camera(name):
    try:
        img = rospy.wait_for_message(name + '/image_raw', Image, timeout=1)
    except rospy.ROSException:
        failure('%s: no images (is the camera connected properly?)', name)
        return
    try:
        info = rospy.wait_for_message(name + '/camera_info', CameraInfo, timeout=1)
    except rospy.ROSException:
        failure('%s: no calibration info', name)
        return

    if img.width != info.width:
        failure('%s: calibration width doesn\'t match image width (%d != %d)', name, info.width, img.width)
    if img.height != info.height:
        failure('%s: calibration height doesn\'t match image height (%d != %d))', name, info.height, img.height)


@check('ArUco detector')
def check_aruco():
    try:
        rospy.wait_for_message('aruco_detect/markers', MarkerArray, timeout=1)
    except rospy.ROSException:
        failure('no markers detection')
        return
    try:
        rospy.wait_for_message('aruco_map/pose', PoseWithCovarianceStamped, timeout=1)
    except rospy.ROSException:
        failure('no map detection')


@check('Vision position estimate')
def check_vpe():
    try:
        vis = rospy.wait_for_message('mavros/vision_pose/pose', PoseStamped, timeout=1)
    except rospy.ROSException:
        try:
            vis = rospy.wait_for_message('mavros/mocap/pose', PoseStamped, timeout=1)
        except rospy.ROSException:
            failure('no VPE or MoCap messages')
            return

    # check vision pose and estimated pose inconsistency
    try:
        pose = rospy.wait_for_message('mavros/local_position/pose', PoseStamped, timeout=1)
    except:
        return
    horiz = math.hypot(vis.pose.position.x - pose.pose.position.x, vis.pose.position.y - pose.pose.position.y)
    if horiz > 0.5:
        failure('horizontal position inconsistency: %.2f m', horiz)
    vert = vis.pose.position.z - pose.pose.position.z
    if abs(vert) > 0.5:
        failure('vertical position inconsistency: %.2f m', vert)
    op = pose.pose.orientation
    ov = vis.pose.orientation
    yawp, _, _ = t.euler_from_quaternion((op.x, op.y, op.z, op.w), axes='rzyx')
    yawv, _, _ = t.euler_from_quaternion((ov.x, ov.y, ov.z, ov.w), axes='rzyx')
    yawdiff = yawp - yawv
    yawdiff = math.degrees((yawdiff + 180) % 360 - 180)
    if abs(yawdiff) > 8:
        failure('yaw inconsistency: %.2f deg', yawdiff)


@check('Simple offboard node')
def check_simpleoffboard():
    try:
        rospy.wait_for_service('navigate', timeout=3)
        rospy.wait_for_service('get_telemetry', timeout=3)
        rospy.wait_for_service('land', timeout=3)
    except rospy.ROSException:
        failure('no simple_offboard services')


@check('IMU')
def check_imu():
    try:
        rospy.wait_for_message('mavros/imu/data', Imu, timeout=1)
    except rospy.ROSException:
        failure('no IMU data (check flight controller calibration)')


@check('Local position')
def check_local_position():
    try:
        pose = rospy.wait_for_message('mavros/local_position/pose', PoseStamped, timeout=1)
        o = pose.pose.orientation
        _, pitch, roll = t.euler_from_quaternion((o.x, o.y, o.z, o.w), axes='rzyx')
        MAX_ANGLE = math.radians(2)
        if abs(pitch) > MAX_ANGLE:
            failure('pitch is %.2f deg; place copter horizontally or redo level horizon calib',
                    math.degrees(pitch))
        if abs(roll) > MAX_ANGLE:
            failure('roll is %.2f deg; place copter horizontally or redo level horizon calib',
                    math.degrees(roll))

    except rospy.ROSException:
        failure('no local position')


@check('Velocity estimation')
def check_velocity():
    try:
        velocity = rospy.wait_for_message('mavros/local_position/velocity', TwistStamped, timeout=1)
        horiz = math.hypot(velocity.twist.linear.x, velocity.twist.linear.y)
        vert = velocity.twist.linear.z
        if abs(horiz) > 0.1:
            failure('horizontal velocity estimation is %.2f m/s; is copter staying still?' % horiz)
        if abs(vert) > 0.1:
            failure('vertical velocity estimation is %.2f m/s; is copter staying still?' % vert)

        angular = velocity.twist.angular
        ANGULAR_VELOCITY_LIMIT = 0.01
        if abs(angular.x) > ANGULAR_VELOCITY_LIMIT:
            failure('pitch rate estimation is %.2f rad/s (%.2f deg/s); is copter staying still?',
                    angular.x, math.degrees(angular.x))
        if abs(angular.y) > ANGULAR_VELOCITY_LIMIT:
            failure('pitch rate estimation is %.2f rad/s (%.2f deg/s); is copter staying still?',
                    angular.y, math.degrees(angular.y))
        if abs(angular.z) > ANGULAR_VELOCITY_LIMIT:
            failure('pitch rate estimation is %.2f rad/s (%.2f deg/s); is copter staying still?',
                    angular.z, math.degrees(angular.z))
    except rospy.ROSException:
        failure('no velocity estimation')


@check('Global position (GPS)')
def check_global_position():
    try:
        rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=1)
    except rospy.ROSException:
        failure('no global position')


@check('Optical flow')
def check_optical_flow():
    # TODO:check FPS!
    try:
        rospy.wait_for_message('mavros/px4flow/raw/send', OpticalFlowRad, timeout=0.5)
    except rospy.ROSException:
        failure('no optical flow data (from Raspberry)')


@check('Rangefinder')
def check_rangefinder():
    # TODO: check FPS!
    try:
        rospy.wait_for_message('mavros/distance_sensor/rangefinder_sub', Range, timeout=0.5)
    except rospy.ROSException:
        failure('no randefinder data from Raspberry')
    try:
        rospy.wait_for_message('mavros/distance_sensor/rangefinder', Range, timeout=0.5)
    except rospy.ROSException:
        failure('no rangefinder data from PX4')


@check('Boot duration')
def check_boot_duration():
    proc = Popen('systemd-analyze', stdout=PIPE)
    proc.wait()
    output = proc.communicate()[0]
    r = re.compile(r'([\d\.]+)s$')
    duration = float(r.search(output).groups()[0])
    if duration > 15:
        failure('long Raspbian boot duration: %ss (systemd-analyze for analyzing)', duration)


@check('CPU usage')
def check_cpu_usage():
    WHITELIST = 'nodelet',
    CMD = "top -n 1 -b -i | tail -n +8 | awk '{ printf(\"%-8s\\t%-8s\\t%-8s\\n\", $1, $9, $12); }'"
    proc = Popen(CMD, stdout=PIPE, shell=True)
    proc.wait()
    output = proc.communicate()[0]
    processes = output.split('\n')
    for process in processes:
        if not process:
            continue
        pid, cpu, cmd = process.split('\t')

        if cmd.strip() not in WHITELIST and float(cpu) > 30:
            failure('high CPU usage (%s%%) detected: %s (PID %s)',
                    cpu.strip(), cmd.strip(), pid.strip())


def selfcheck():
    check_fcu()
    check_imu()
    check_local_position()
    check_velocity()
    check_global_position()
    check_camera('main_camera')
    check_aruco()
    check_simpleoffboard()
    check_optical_flow()
    check_vpe()
    check_rangefinder()
    check_cpu_usage()
    check_boot_duration()


if __name__ == '__main__':
    rospy.loginfo('Performing selfcheck...')
    selfcheck()
