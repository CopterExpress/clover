#!/usr/bin/env python
# coding=utf-8

# Copyright (C) 2018 Copter Express Technologies
#
# Author: Oleg Kalachev <okalachev@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

import os
import math
import subprocess
import re
from collections import OrderedDict
import traceback
from threading import Event
import numpy
import rospy
import tf2_ros
import tf2_geometry_msgs
from pymavlink import mavutil
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState, Image, CameraInfo, NavSatFix, Imu, Range
from mavros_msgs.msg import State, OpticalFlowRad, Mavlink
from mavros_msgs.srv import ParamGet
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, Vector3Stamped
from visualization_msgs.msg import MarkerArray as VisualizationMarkerArray
import tf.transformations as t
from aruco_pose.msg import MarkerArray
from mavros import mavlink


# TODO: check attitude is present
# TODO: disk free space
# TODO: map, base_link, body
# TODO: rc service
# TODO: perform commander check, ekf2 status on PX4
# TODO: check if FCU params setter succeed
# TODO: selfcheck ROS service (with blacklists for checks)


rospy.init_node('selfcheck')

os.environ['ROSCONSOLE_FORMAT']='[${severity}]: ${message}'


tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)


failures = []
infos = []
current_check = None


def failure(text, *args):
    msg = text % args
    rospy.logwarn('%s: %s', current_check, msg)
    failures.append(msg)


def info(text, *args):
    msg = text % args
    rospy.loginfo('%s: %s', current_check, msg)
    infos.append(msg)


def check(name):
    def inner(fn):
        def wrapper(*args, **kwargs):
            failures[:] = []
            infos[:] = []
            global current_check
            current_check = name
            try:
                fn(*args, **kwargs)
            except Exception as e:
                traceback.print_exc()
                rospy.logerr('%s: exception occurred', name)
                return
            if not failures and not infos:
                rospy.loginfo('%s: OK', name)
        return wrapper
    return inner


param_get = rospy.ServiceProxy('mavros/param/get', ParamGet)


def get_param(name):
    try:
        res = param_get(param_id=name)
    except rospy.ServiceException as e:
        failure('%s: %s', name, str(e))
        return None

    if not res.success:
        failure('unable to retrieve PX4 parameter %s', name)
    else:
        if res.value.integer != 0:
            return res.value.integer
        return res.value.real


recv_event = Event()
link = mavutil.mavlink.MAVLink('', 255, 1)
mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)
mavlink_recv = ''


def mavlink_message_handler(msg):
    global mavlink_recv
    if msg.msgid == 126:
        mav_bytes_msg = mavlink.convert_to_bytes(msg)
        mav_msg = link.decode(mav_bytes_msg)
        mavlink_recv += ''.join(chr(x) for x in mav_msg.data[:mav_msg.count])
        if 'nsh>' in mavlink_recv:
            # Remove the last line, including newline before prompt
            mavlink_recv = mavlink_recv[:mavlink_recv.find('nsh>') - 1]
            recv_event.set()


mavlink_sub = rospy.Subscriber('mavlink/from', Mavlink, mavlink_message_handler)
# FIXME: not sleeping here still breaks things
rospy.sleep(0.5)


def mavlink_exec(cmd, timeout=3.0):
    global mavlink_recv
    mavlink_recv = ''
    recv_event.clear()
    if not cmd.endswith('\n'):
        cmd += '\n'
    msg = mavutil.mavlink.MAVLink_serial_control_message(
        device=mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
        flags=mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
              mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI,
        timeout=3,
        baudrate=0,
        count=len(cmd),
        data=[ord(c) for c in cmd.ljust(70, '\0')])
    msg.pack(link)
    ros_msg = mavlink.convert_to_rosmsg(msg)
    mavlink_pub.publish(ros_msg)
    recv_event.wait(timeout)
    return mavlink_recv


BOARD_ROTATIONS = {
    0: 'no rotation',
    1: 'yaw 45°',
    2: 'yaw 90°',
    3: 'yaw 135°',
    4: 'yaw 180°',
    5: 'yaw 225°',
    6: 'yaw 270°',
    7: 'yaw 315°',
    8: 'roll 180°',
    9: 'roll 180°, yaw 45°',
    10: 'roll 180°, yaw 90°',
    11: 'roll 180°, yaw 135°',
    12: 'pitch 180°',
    13: 'roll 180°, yaw 225°',
    14: 'roll 180°, yaw 270°',
    15: 'roll 180°, yaw 315°',
    16: 'roll 90°',
    17: 'roll 90°, yaw 45°',
    18: 'roll 90°, yaw 90°',
    19: 'roll 90°, yaw 135°',
    20: 'roll 270°',
    21: 'roll 270°, yaw 45°',
    22: 'roll 270°, yaw 90°',
    23: 'roll 270°, yaw 135°',
    24: 'pitch 90°',
    25: 'pitch 270°',
    26: 'roll 270°, yaw 270°',
    27: 'roll 180°, pitch 270°',
    28: 'pitch 90°, yaw 180',
    29: 'pitch 90°, roll 90°',
    30: 'yaw 293°, pitch 68°, roll 90°',
    31: 'pitch 90°, roll 270°',
    32: 'pitch 9°, yaw 180°',
    33: 'pitch 45°',
    34: 'pitch 315°',
}


@check('FCU')
def check_fcu():
    try:
        state = rospy.wait_for_message('mavros/state', State, timeout=3)
        if not state.connected:
            failure('no connection to the FCU (check wiring)')
            return

        # Make sure the console is available to us
        mavlink_exec('\n')
        version_str = mavlink_exec('ver all')
        if version_str == '':
            info('no version data available from SITL')

        r = re.compile(r'^FW (git tag|version): (v?\d\.\d\.\d.*)$')
        is_clover_firmware = False
        for ver_line in version_str.split('\n'):
            match = r.search(ver_line)
            if match is not None:
                field, version = match.groups()
                info('firmware %s: %s' % (field, version))
                if 'clover' in version or 'clever' in version:
                    is_clover_firmware = True

        if not is_clover_firmware:
            failure('not running Clover PX4 firmware, https://clover.coex.tech/firmware')

        est = get_param('SYS_MC_EST_GROUP')
        if est == 1:
            info('selected estimator: LPE')
            fuse = get_param('LPE_FUSION')
            if fuse & (1 << 4):
                info('LPE_FUSION: land detector fusion is enabled')
            else:
                info('LPE_FUSION: land detector fusion is disabled')
            if fuse & (1 << 7):
                info('LPE_FUSION: barometer fusion is enabled')
            else:
                info('LPE_FUSION: barometer fusion is disabled')

            mag_yaw_w = get_param('ATT_W_MAG')
            if mag_yaw_w == 0:
                info('magnetometer weight (ATT_W_MAG) is zero, better for indoor flights')
            else:
                info('magnetometer weight (ATT_W_MAG) is non-zero (%.2f), better for outdoor flights', mag_yaw_w)

        elif est == 2:
            info('selected estimator: EKF2')
        else:
            failure('unknown selected estimator: %s', est)

        rot = get_param('SENS_BOARD_ROT')
        if rot is not None:
            try:
                info('board rotation: %s', BOARD_ROTATIONS[rot])
            except KeyError:
                failure('unknown board rotation %s', rot)

        cbrk_usb_chk = get_param('CBRK_USB_CHK')
        if cbrk_usb_chk != 197848:
            failure('set parameter CBRK_USB_CHK to 197848 for flying with USB connected')

        try:
            battery = rospy.wait_for_message('mavros/battery', BatteryState, timeout=3)
            if not battery.cell_voltage:
                failure('cell voltage is not available, https://clover.coex.tech/power')
            else:
                cell = battery.cell_voltage[0]
                if cell > 4.3 or cell < 3.0:
                    failure('incorrect cell voltage: %.2f V, https://clover.coex.tech/power', cell)
                elif cell < 3.7:
                    failure('critically low cell voltage: %.2f V, recharge battery', cell)
        except rospy.ROSException:
            failure('no battery state')

    except rospy.ROSException:
        failure('no MAVROS state (check wiring)')


def describe_direction(v):
    if v.x > 0.9:
        return 'forward'
    elif v.x < - 0.9:
        return 'backward'
    elif v.y > 0.9:
        return 'left'
    elif v.y < -0.9:
        return 'right'
    elif v.z > 0.9:
        return 'upward'
    elif v.z < -0.9:
        return 'downward'
    else:
        return None


def check_camera(name):
    try:
        img = rospy.wait_for_message(name + '/image_raw', Image, timeout=1)
    except rospy.ROSException:
        failure('%s: no images (is the camera connected properly?)', name)
        return
    try:
        camera_info = rospy.wait_for_message(name + '/camera_info', CameraInfo, timeout=1)
    except rospy.ROSException:
        failure('%s: no calibration info', name)
        return

    if img.width != camera_info.width:
        failure('%s: calibration width doesn\'t match image width (%d != %d)', name, camera_info.width, img.width)
    if img.height != camera_info.height:
        failure('%s: calibration height doesn\'t match image height (%d != %d))', name, camera_info.height, img.height)

    try:
        optical = Vector3Stamped()
        optical.header.frame_id = img.header.frame_id
        optical.vector.z = 1
        cable = Vector3Stamped()
        cable.header.frame_id = img.header.frame_id
        cable.vector.y = 1

        optical = describe_direction(tf_buffer.transform(optical, 'base_link').vector)
        cable = describe_direction(tf_buffer.transform(cable, 'base_link').vector)
        if not optical or not cable:
            info('%s: custom camera orientation detected', name)
        else:
            info('camera is oriented %s, cable from camera goes %s', optical, cable)

    except tf2_ros.TransformException:
        failure('cannot transform from base_link to camera frame')


@check('Main camera')
def check_main_camera():
    check_camera('main_camera')


def is_process_running(binary, exact=False, full=False):
    try:
        args = ['pgrep']
        if exact:
            args.append('-x')  # match exactly with the command name
        if full:
            args.append('-f')  # use full process name to match
        args.append(binary)
        subprocess.check_output(args)
        return True
    except subprocess.CalledProcessError:
        return False


@check('ArUco markers')
def check_aruco():
    if is_process_running('aruco_detect', full=True):
        try:
            info('aruco_detect/length = %g m', rospy.get_param('aruco_detect/length'))
        except KeyError:
            failure('aruco_detect/length parameter is not set')
        known_tilt = rospy.get_param('aruco_detect/known_tilt', '')
        if known_tilt == 'map':
            known_tilt += ' (ALL markers are on the floor)'
        elif known_tilt == 'map_flipped':
            known_tilt += ' (ALL markers are on the ceiling)'
        info('aruco_detector/known_tilt = %s', known_tilt)
        try:
            rospy.wait_for_message('aruco_detect/markers', MarkerArray, timeout=1)
        except rospy.ROSException:
            failure('no markers detection')
            return
    else:
        info('aruco_detect is not running')
        return

    if is_process_running('aruco_map', full=True):
        known_tilt = rospy.get_param('aruco_map/known_tilt', '')
        if known_tilt == 'map':
            known_tilt += ' (marker\'s map is on the floor)'
        elif known_tilt == 'map_flipped':
            known_tilt += ' (marker\'s map is on the ceiling)'
        info('aruco_map/known_tilt = %s', known_tilt)

        try:
            visualization = rospy.wait_for_message('aruco_map/visualization', VisualizationMarkerArray, timeout=1)
            info('map has %s markers', len(visualization.markers))
        except:
            failure('cannot read aruco_map/visualization topic')

        try:
            rospy.wait_for_message('aruco_map/pose', PoseWithCovarianceStamped, timeout=1)
        except rospy.ROSException:
            failure('no map detection')
    else:
        info('aruco_map is not running')


@check('Vision position estimate')
def check_vpe():
    vis = None
    try:
        vis = rospy.wait_for_message('mavros/vision_pose/pose', PoseStamped, timeout=1)
    except rospy.ROSException:
        try:
            vis = rospy.wait_for_message('mavros/mocap/pose', PoseStamped, timeout=1)
        except rospy.ROSException:
            failure('no VPE or MoCap messages')
            # check if vpe_publisher is running
            try:
                subprocess.check_output(['pgrep', '-x', 'vpe_publisher'])
            except subprocess.CalledProcessError:
                return  # it's not running, skip following checks

    # check PX4 settings
    est = get_param('SYS_MC_EST_GROUP')
    if est == 1:
        ext_yaw = get_param('ATT_EXT_HDG_M')
        if ext_yaw != 1:
            failure('vision yaw is disabled, change ATT_EXT_HDG_M parameter')
        vision_yaw_w = get_param('ATT_W_EXT_HDG')
        if vision_yaw_w == 0:
            failure('vision yaw weight is zero, change ATT_W_EXT_HDG parameter')
        else:
            info('Vision yaw weight: %.2f', vision_yaw_w)
        fuse = get_param('LPE_FUSION')
        if not fuse & (1 << 2):
            failure('vision position fusion is disabled, change LPE_FUSION parameter')
        delay = get_param('LPE_VIS_DELAY')
        if delay != 0:
            failure('LPE_VIS_DELAY parameter is %s, but it should be zero', delay)
        info('LPE_VIS_XY is %.2f m, LPE_VIS_Z is %.2f m', get_param('LPE_VIS_XY'), get_param('LPE_VIS_Z'))
    elif est == 2:
        fuse = get_param('EKF2_AID_MASK')
        if not fuse & (1 << 3):
            failure('vision position fusion is disabled, change EKF2_AID_MASK parameter')
        if not fuse & (1 << 4):
            failure('vision yaw fusion is disabled, change EKF2_AID_MASK parameter')
        delay = get_param('EKF2_EV_DELAY')
        if delay != 0:
            failure('EKF2_EV_DELAY is %.2f, but it should be zero', delay)
        info('EKF2_EVA_NOISE is %.3f, EKF2_EVP_NOISE is %.3f',
            get_param('EKF2_EVA_NOISE'),
            get_param('EKF2_EVP_NOISE'))

    if not vis:
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

        if not tf_buffer.can_transform('base_link', pose.header.frame_id, rospy.get_rostime(), rospy.Duration(0.5)):
            failure('can\'t transform from %s to base_link (timeout 0.5 s): is TF enabled?', pose.header.frame_id)

    except rospy.ROSException:
        failure('no local position')


@check('Velocity estimation')
def check_velocity():
    try:
        velocity = rospy.wait_for_message('mavros/local_position/velocity_local', TwistStamped, timeout=1)
        horiz = math.hypot(velocity.twist.linear.x, velocity.twist.linear.y)
        vert = velocity.twist.linear.z
        if abs(horiz) > 0.1:
            failure('horizontal velocity estimation is %.2f m/s; is copter staying still?' % horiz)
        if abs(vert) > 0.1:
            failure('vertical velocity estimation is %.2f m/s; is copter staying still?' % vert)

        velocity = rospy.wait_for_message('mavros/local_position/velocity_body', TwistStamped, timeout=1)
        angular = velocity.twist.angular
        ANGULAR_VELOCITY_LIMIT = 0.1
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
        info('no global position')


@check('Optical flow')
def check_optical_flow():
    # TODO:check FPS!
    try:
        rospy.wait_for_message('mavros/px4flow/raw/send', OpticalFlowRad, timeout=0.5)

        # check PX4 settings
        rot = get_param('SENS_FLOW_ROT')
        if rot != 0:
            failure('SENS_FLOW_ROT parameter is %s, but it should be zero', rot)
        est = get_param('SYS_MC_EST_GROUP')
        if est == 1:
            fuse = get_param('LPE_FUSION')
            if not fuse & (1 << 1):
                failure('optical flow fusion is disabled, change LPE_FUSION parameter')
            if not fuse & (1 << 1):
                failure('flow gyro compensation is disabled, change LPE_FUSION parameter')
            scale = get_param('LPE_FLW_SCALE')
            if not numpy.isclose(scale, 1.0):
                failure('LPE_FLW_SCALE parameter is %.2f, but it should be 1.0', scale)

            info('LPE_FLW_QMIN is %s, LPE_FLW_R is %.4f, LPE_FLW_RR is %.4f, SENS_FLOW_MINHGT is %.3f, SENS_FLOW_MAXHGT is %.3f',
                          get_param('LPE_FLW_QMIN'),
                          get_param('LPE_FLW_R'),
                          get_param('LPE_FLW_RR'),
                          get_param('SENS_FLOW_MINHGT'),
                          get_param('SENS_FLOW_MAXHGT'))
        elif est == 2:
            fuse = get_param('EKF2_AID_MASK')
            if not fuse & (1 << 1):
                failure('optical flow fusion is disabled, change EKF2_AID_MASK parameter')
            delay = get_param('EKF2_OF_DELAY')
            if delay != 0:
                failure('EKF2_OF_DELAY is %.2f, but it should be zero', delay)
            info('EKF2_OF_QMIN is %s, EKF2_OF_N_MIN is %.4f, EKF2_OF_N_MAX is %.4f, SENS_FLOW_MINHGT is %.3f, SENS_FLOW_MAXHGT is %.3f',
                          get_param('EKF2_OF_QMIN'),
                          get_param('EKF2_OF_N_MIN'),
                          get_param('EKF2_OF_N_MAX'),
                          get_param('SENS_FLOW_MINHGT'),
                          get_param('SENS_FLOW_MAXHGT'))

    except rospy.ROSException:
        failure('no optical flow data (from Raspberry)')


@check('Rangefinder')
def check_rangefinder():
    # TODO: check FPS!
    rng = False
    try:
        rospy.wait_for_message('rangefinder/range', Range, timeout=4)
        rng = True
    except rospy.ROSException:
        failure('no rangefinder data from Raspberry')

    try:
        rospy.wait_for_message('mavros/distance_sensor/rangefinder', Range, timeout=4)
        rng = True
    except rospy.ROSException:
        failure('no rangefinder data from PX4')

    if not rng:
        return

    est = get_param('SYS_MC_EST_GROUP')
    if est == 1:
        fuse = get_param('LPE_FUSION')
        if not fuse & (1 << 5):
            info('"pub agl as lpos down" in LPE_FUSION is disabled, NOT operating over flat surface')
        else:
            info('"pub agl as lpos down" in LPE_FUSION is enabled, operating over flat surface')

    elif est == 2:
        hgt = get_param('EKF2_HGT_MODE')
        if hgt != 2:
            info('EKF2_HGT_MODE != Range sensor, NOT operating over flat surface')
        else:
            info('EKF2_HGT_MODE = Range sensor, operating over flat surface')
        aid = get_param('EKF2_RNG_AID')
        if aid != 1:
            info('EKF2_RNG_AID != 1, range sensor aiding disabled')
        else:
            info('EKF2_RNG_AID = 1, range sensor aiding enabled')


@check('Boot duration')
def check_boot_duration():
    output = subprocess.check_output('systemd-analyze').decode()
    r = re.compile(r'([\d\.]+)s\s*$', flags=re.MULTILINE)
    duration = float(r.search(output).groups()[0])
    if duration > 20:
        failure('long Raspbian boot duration: %ss (systemd-analyze for analyzing)', duration)


@check('CPU usage')
def check_cpu_usage():
    WHITELIST = 'nodelet', 'gzclient', 'gzserver'
    CMD = "top -n 1 -b -i | tail -n +8 | awk '{ printf(\"%-8s\\t%-8s\\t%-8s\\n\", $1, $9, $12); }'"
    output = subprocess.check_output(CMD, shell=True).decode()
    processes = output.split('\n')
    for process in processes:
        if not process:
            continue
        pid, cpu, cmd = process.split('\t')

        if cmd.strip() not in WHITELIST and float(cpu) > 30:
            failure('high CPU usage (%s%%) detected: %s (PID %s)',
                    cpu.strip(), cmd.strip(), pid.strip())


@check('clover.service')
def check_clover_service():
    try:
        output = subprocess.check_output('systemctl show -p ActiveState --value clover.service'.split(),
                                         stderr=subprocess.STDOUT).decode()
    except subprocess.CalledProcessError as e:
        failure('systemctl returned %s: %s', e.returncode, e.output)
        return
    if 'inactive' in output:
        failure('service is not running, try sudo systemctl restart clover')
        return
    elif 'failed' in output:
        failure('service failed to run, check your launch-files')

    BLACKLIST = 'Unexpected command 520', 'Time jump detected', 'different index:'

    r = re.compile(r'^(.*)\[(FATAL|ERROR| WARN)\] \[\d+.\d+\]: (.*?)(\x1b(.*))?$')
    error_count = OrderedDict()
    try:
        for line in open('/tmp/clover.err', 'r'):
            skip = False
            for substr in BLACKLIST:
                if substr in line:
                    skip = True
            if skip:
                continue

            node_error = r.search(line)
            if node_error:
                msg = node_error.groups()[1].strip() + ': ' + node_error.groups()[2]
                if msg in error_count:
                    error_count[msg] += 1
                else:
                    error_count.update({msg: 1})
            else:
                error_count.update({line.strip(): 1})

        for error in error_count:
            if error_count[error] == 1:
                failure(error)
            else:
                failure('%s (%d)', error, error_count[error])

    except IOError as e:
        failure('%s', e)


@check('Image')
def check_image():
    try:
        info('version: %s', open('/etc/clover_version').read().strip())
    except IOError:
        info('no /etc/clover_version file, not the Clover image?')


@check('Preflight status')
def check_preflight_status():
    # Make sure the console is available to us
    mavlink_exec('\n')
    cmdr_output = mavlink_exec('commander check')
    if cmdr_output == '':
        failure('no data from FCU')
        return
    cmdr_lines = cmdr_output.split('\n')
    r = re.compile(r'^(.*)(Preflight|Prearm) check: (.*)')
    for line in cmdr_lines:
        if 'WARN' in line:
            failure(line[line.find(']') + 2:])
            continue
        match = r.search(line)
        if match is not None:
            check_status = match.groups()[2]
            if check_status != 'OK':
                failure(' '.join([match.groups()[1], 'check:', check_status]))


@check('Network')
def check_network():
    ros_hostname = os.environ.get('ROS_HOSTNAME', '').strip()

    if not ros_hostname:
        failure('no ROS_HOSTNAME is set')

    elif ros_hostname.endswith('.local'):
        # using mdns hostname
        hosts = open('/etc/hosts', 'r')
        for line in hosts:
            parts = line.split()
            if len(parts) < 2:
                continue
            ip = parts.pop(0).split('.')
            if ip[0] == '127':  # loopback ip
                if ros_hostname in parts:
                    break
        else:
            failure('not found %s in /etc/hosts, ROS will malfunction if network interfaces are down, https://clover.coex.tech/hostname', ros_hostname)


@check('RPi health')
def check_rpi_health():
    # `vcgencmd get_throttled` output codes taken from
    # https://github.com/raspberrypi/documentation/blob/JamesH65-patch-vcgencmd-vcdbg-docs/raspbian/applications/vcgencmd.md#get_throttled
    # TODO: support more base platforms?
    FLAG_UNDERVOLTAGE_NOW = 0x1
    FLAG_FREQ_CAP_NOW = 0x2
    FLAG_THROTTLING_NOW = 0x4
    FLAG_THERMAL_LIMIT_NOW = 0x8
    FLAG_UNDERVOLTAGE_OCCURRED = 0x10000
    FLAG_FREQ_CAP_OCCURRED = 0x20000
    FLAG_THROTTLING_OCCURRED = 0x40000
    FLAG_THERMAL_LIMIT_OCCURRED = 0x80000

    FLAG_DESCRIPTIONS = (
        (FLAG_THROTTLING_NOW, 'system throttled to prevent damage'),
        (FLAG_THROTTLING_OCCURRED, 'your system is susceptible to throttling'),
        (FLAG_UNDERVOLTAGE_NOW, 'not enough power for onboard computer, flight inadvisable'),
        (FLAG_UNDERVOLTAGE_OCCURRED, 'power supply cannot provide enough power'),
        (FLAG_FREQ_CAP_NOW, 'CPU reached thermal limit and is throttled now'),
        (FLAG_FREQ_CAP_OCCURRED, 'CPU may overheat during drone operation, consider additional cooling'),
        (FLAG_THERMAL_LIMIT_NOW, 'CPU reached soft thermal limit, frequency reduced'),
        (FLAG_THERMAL_LIMIT_OCCURRED, 'CPU may reach soft thermal limit, consider additional cooling'),
    )

    try:
        # vcgencmd outputs a single string in a form of
        # <parameter>=<value>
        # In case of `get_throttled`, <value> is a hexadecimal number
        # with some of the FLAGs OR'ed together
        output = subprocess.check_output(['vcgencmd', 'get_throttled']).decode()
    except OSError:
        info('could not call vcgencmd binary; not a Raspberry Pi?')
        return

    throttle_mask = int(output.split('=')[1], base=16)
    for flag_description in FLAG_DESCRIPTIONS:
        if throttle_mask & flag_description[0]:
            failure(flag_description[1])


@check('Board')
def check_board():
    try:
        info('%s', open('/proc/device-tree/model').readline())
    except IOError:
        info('could not open /proc/device-tree/model, not a Raspberry Pi?')


def selfcheck():
    check_image()
    check_board()
    check_clover_service()
    check_network()
    check_fcu()
    check_imu()
    check_local_position()
    check_velocity()
    check_global_position()
    check_preflight_status()
    check_main_camera()
    check_aruco()
    check_simpleoffboard()
    check_optical_flow()
    check_vpe()
    check_rangefinder()
    check_rpi_health()
    check_cpu_usage()
    check_boot_duration()


if __name__ == '__main__':
    rospy.loginfo('Performing selfcheck...')
    selfcheck()
