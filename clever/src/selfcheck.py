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


# TODO: roscore is running
# TODO: clever.service is running
# TODO: check attitude is present
# TODO: disk free space
# TODO: local_origin, fcu, fcu_horiz
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
                rospy.logwarn('%s: exception occured', name)
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
            failure('No connection to the FCU (check wiring)')
    except rospy.ROSException:
        failure('No MAVROS state (check wiring)')


@check('Camera')
def check_camera(name):
    try:
        img = rospy.wait_for_message(name + '/image_raw', Image, timeout=1)
    except rospy.ROSException:
        failure('%s: No images (is the camera connected properly?)', name)
        return
    try:
        info = rospy.wait_for_message(name + '/camera_info', CameraInfo, timeout=1)
    except rospy.ROSException:
        failure('%s: No calibration info', name)
        return

    if img.width != info.width:
        failure('%s: Calibration width doesn\'t match image width (%d != %d)', name, info.width, img.width)
    if img.height != info.height:
        failure('%s: Calibration height doesn\'t match image height (%d != %d))', name, info.height, img.height)


@check('Aruco detector')
def check_aruco():
    try:
        rospy.wait_for_message('aruco_pose/debug', Image, timeout=1)
    except rospy.ROSException:
        failure('No aruco_pose/debug messages')


@check('Visual position estimate')
def check_vpe():
    try:
        rospy.wait_for_message('mavros/vision_pose/pose', PoseStamped, timeout=1)
    except rospy.ROSException:
        try:
            rospy.wait_for_message('mavros/mocap/pose', PoseStamped, timeout=1)
        except rospy.ROSException:
            failure('No VPE or MoCap messages')


@check('Simple offboard node')
def check_simpleoffboard():
    try:
        rospy.wait_for_service('navigate', timeout=3)
        rospy.wait_for_service('get_telemetry', timeout=3)
        rospy.wait_for_service('land', timeout=3)
    except rospy.ROSException:
        failure('No simple_offboard services')


@check('IMU')
def check_imu():
    try:
        rospy.wait_for_message('mavros/imu/data', Imu, timeout=1)
    except rospy.ROSException:
        failure('No IMU data (check flight controller calibration)')


@check('Local position')
def check_local_position():
    try:
        rospy.wait_for_message('mavros/local_position/pose', PoseStamped, timeout=1)
    except rospy.ROSException:
        failure('No local position')


@check('Velocity estimation')
def check_velocity():
    try:
        velocity = rospy.wait_for_message('mavros/local_position/velocity', TwistStamped, timeout=1)
        horiz = math.hypot(velocity.twist.linear.x, velocity.twist.linear.y)
        vert = velocity.twist.linear.z
        if abs(horiz) > 0.1:
            failure('Horizontal velocity estimation is %.2f m/s; is copter staying still?' % horiz)
        if abs(vert) > 0.1:
            failure('Vertical velocity estimation is %.2f m/s; is copter staying still?' % vert)

        angular = velocity.twist.angular
        ANGULAR_VELOCITY_LIMIT = 0.01
        if abs(angular.x) > ANGULAR_VELOCITY_LIMIT:
            failure('Pitch rate estimation is %.2f rad/s (%.2f deg/s); is copter staying still?',
                    angular.x, math.degrees(angular.x))
        if abs(angular.y) > ANGULAR_VELOCITY_LIMIT:
            failure('Pitch rate estimation is %.2f rad/s (%.2f deg/s); is copter staying still?',
                    angular.y, math.degrees(angular.y))
        if abs(angular.z) > ANGULAR_VELOCITY_LIMIT:
            failure('Pitch rate estimation is %.2f rad/s (%.2f deg/s); is copter staying still?',
                    angular.z, math.degrees(angular.z))
    except rospy.ROSException:
        failure('No velocity estimation')


@check('Global position (GPS)')
def check_global_position():
    try:
        rospy.wait_for_message('mavros/global_position/global', NavSatFix, timeout=1)
    except rospy.ROSException:
        failure('No global position')


@check('Optical flow')
def check_optical_flow():
    # TODO:check FPS!
    try:
        rospy.wait_for_message('mavros/px4flow/raw/send', OpticalFlowRad, timeout=0.5)
    except rospy.ROSException:
        failure('No optical flow data (from Raspberry)')


@check('Rangefinder')
def check_rangefinder():
    # TODO: check FPS!
    try:
        rospy.wait_for_message('mavros/distance_sensor/rangefinder_3_sub', Range, timeout=0.5)
    except rospy.ROSException:
        failure('No randefinder data from Raspberry')
    try:
        rospy.wait_for_message('mavros/distance_sensor/rangefinder_0', Range, timeout=0.5)
    except rospy.ROSException:
        failure('No rangefinder data from PX4')


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
            failure('High CPU usage (%s%%) detected: %s (PID %s)',
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
