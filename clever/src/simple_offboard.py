#!/usr/bin/env python
from __future__ import division

import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, PointStamped, Vector3, \
    Vector3Stamped, TwistStamped, QuaternionStamped
from sensor_msgs.msg import NavSatFix, BatteryState
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


position_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
target_pub = rospy.Publisher('~target', PoseStamped, queue_size=1)

arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool, persistent=True)
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode, persistent=True)


pose = None
global_position = None
velocity = None
state = None
battery = None


def pose_update(data):
    global pose
    pose = data


def global_position_update(data):
    global global_position
    global_position = data


def velocity_update(data):
    global velocity
    velocity = data


def state_update(data):
    global state
    state = data


def battery_update(data):
    global battery
    battery = data


rospy.Subscriber('/mavros/state', State, state_update)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_update)
rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, velocity_update)
rospy.Subscriber('/mavros/global_position/global', NavSatFix, global_position_update)
rospy.Subscriber('/mavros/battery', BatteryState, battery_update)


PT = PositionTarget
AT = AttitudeTarget
AUTO_OFFBOARD = rospy.get_param('~auto_offboard', True)
AUTO_ARM = AUTO_OFFBOARD and rospy.get_param('~auto_arm', True)
OFFBOARD_TIMEOUT = rospy.Duration(rospy.get_param('~offboard_timeout', 3))
ARM_TIMEOUT = rospy.Duration(rospy.get_param('~arm_timeout', 5))
LOCAL_POSITION_TIMEOUT = rospy.Duration(rospy.get_param('~local_position_timeout', 0.5))
NAVIGATE_AFTER_ARMED = rospy.Duration(rospy.get_param('~navigate_after_armed', True))
TRANSFORM_TIMEOUT = rospy.Duration(rospy.get_param('~transform_timeout', 3))
SETPOINT_RATE = rospy.get_param('~setpoint_rate', 30)
LOCAL_FRAME = rospy.get_param('mavros/local_position/frame_id', 'local_origin')
LAND_MODE = rospy.get_param('~land_mode', 'AUTO.LAND')
LAND_TIMEOUT = rospy.Duration(rospy.get_param('~land_timeout', 2))
DEFAULT_SPEED = rospy.get_param('~default_speed', 0.5)


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
            rospy.sleep(0.1)

    if AUTO_ARM and not state.armed:
        rospy.loginfo('Arming')
        res = arming(True)

        start = rospy.get_rostime()
        while True:
            if state.armed:
                return True
            if rospy.get_rostime() - start > ARM_TIMEOUT:
                raise Exception('Arming timed out')
            rospy.sleep(0.1)


ps = PoseStamped()
vs = Vector3Stamped()
pt = PositionTarget()
at = AttitudeTarget()


BRAKE_TIME = rospy.Duration(0)


def get_navigate_setpoint(stamp, start, finish, start_stamp, speed):
    distance = math.sqrt((finish.z - start.z)**2 + (finish.x - start.x)**2 + (finish.y - start.y)**2)
    time = rospy.Duration(distance / speed)
    if time == rospy.Duration(0):
        k = 0
    else:
        k = (stamp - start_stamp) / time
    time_left = start_stamp + time - stamp

    if BRAKE_TIME and time_left < BRAKE_TIME:
        # time to brake
        time_before_braking = time - BRAKE_TIME
        brake_time_passed = (stamp - start_stamp - time_before_braking)

        if brake_time_passed > 2 * BRAKE_TIME:
            # finish
            k = 1
        else:
            # brake!
            k_before_braking = time_before_braking / time
            k_after_braking = (speed * brake_time_passed.to_sec() - brake_time_passed.to_sec() ** 2 * speed / 4 / BRAKE_TIME.to_sec()) / distance
            k = k_before_braking + k_after_braking

    k = min(k, 1)

    p = Point()
    p.x = start.x + (finish.x - start.x) * k
    p.y = start.y + (finish.y - start.y) * k
    p.z = start.z + (finish.z - start.z) * k
    return p


def get_publisher_and_message(req, stamp, continued=True, update_frame=True):
    ps.header.stamp = stamp
    vs.header.stamp = stamp

    # don't block on setpoints publishing
    transform_timeout = rospy.Duration(0.1) if continued else TRANSFORM_TIMEOUT

    if isinstance(req, (srv.NavigateRequest, srv.NavigateGlobalRequest)):
        global current_nav_start, current_nav_start_stamp, current_nav_finish

        if update_frame:
            ps.header.frame_id = req.frame_id or LOCAL_FRAME
            ps.pose.position = Point(getattr(req, 'x', 0), getattr(req, 'y', 0), req.z)
            ps.pose.orientation = orientation_from_euler(0, 0, req.yaw, axes='sxyz')
            current_nav_finish = tf_buffer.transform(ps, LOCAL_FRAME, transform_timeout)

            if isinstance(req, srv.NavigateGlobalRequest):
                # Recalculate x and y from lat and lon
                current_nav_finish.pose.position.x, current_nav_finish.pose.position.y = \
                    global_to_local(req.lat, req.lon)

        if not continued:
            current_nav_start = pose.pose.position
            current_nav_start_stamp = stamp

        if NAVIGATE_AFTER_ARMED and not state.armed:
            current_nav_start_stamp = stamp

        setpoint = get_navigate_setpoint(stamp, current_nav_start, current_nav_finish.pose.position,
                                         current_nav_start_stamp, req.speed)

        yaw_rate_flag = math.isnan(req.yaw)
        msg = pt
        msg.coordinate_frame = PT.FRAME_LOCAL_NED
        msg.type_mask = PT.IGNORE_VX + PT.IGNORE_VY + PT.IGNORE_VZ + \
                        PT.IGNORE_AFX + PT.IGNORE_AFY + PT.IGNORE_AFZ + \
                        (PT.IGNORE_YAW if yaw_rate_flag else PT.IGNORE_YAW_RATE)
        msg.position = setpoint
        msg.yaw = euler_from_orientation(current_nav_finish.pose.orientation, 'sxyz')[2]
        msg.yaw_rate = req.yaw_rate
        return position_pub, msg

    elif isinstance(req, (srv.SetPositionRequest, srv.SetPositionGlobalRequest)):
        ps.header.frame_id = req.frame_id or LOCAL_FRAME
        ps.pose.position = Point(getattr(req, 'x', 0), getattr(req, 'y', 0), req.z)
        ps.pose.orientation = orientation_from_euler(0, 0, req.yaw)
        pose_local = tf_buffer.transform(ps, LOCAL_FRAME, transform_timeout)

        if isinstance(req, srv.SetPositionGlobalRequest):
            pose_local.pose.position.x, pose_local.pose.position.y = global_to_local(req.lat, req.lon)

        yaw_rate_flag = math.isnan(req.yaw)
        msg = pt
        msg.coordinate_frame = PT.FRAME_LOCAL_NED
        msg.type_mask = PT.IGNORE_VX + PT.IGNORE_VY + PT.IGNORE_VZ + \
                        PT.IGNORE_AFX + PT.IGNORE_AFY + PT.IGNORE_AFZ + \
                        (PT.IGNORE_YAW if yaw_rate_flag else PT.IGNORE_YAW_RATE)
        msg.position = pose_local.pose.position
        msg.yaw = euler_from_orientation(pose_local.pose.orientation, 'sxyz')[2]
        msg.yaw_rate = req.yaw_rate
        return position_pub, msg

    elif isinstance(req, srv.SetVelocityRequest):
        vs.vector = Vector3(req.vx, req.vy, req.vz)
        vs.header.frame_id = req.frame_id or LOCAL_FRAME
        ps.header.frame_id = req.frame_id or LOCAL_FRAME
        ps.pose.orientation = orientation_from_euler(0, 0, req.yaw)
        pose_local = tf_buffer.transform(ps, LOCAL_FRAME, transform_timeout)
        vector_local = tf_buffer.transform(vs, LOCAL_FRAME, transform_timeout)

        yaw_rate_flag = math.isnan(req.yaw)
        msg = pt
        msg.coordinate_frame = PT.FRAME_LOCAL_NED
        msg.type_mask = PT.IGNORE_PX + PT.IGNORE_PY + PT.IGNORE_PZ + \
                                       PT.IGNORE_AFX + PT.IGNORE_AFY + PT.IGNORE_AFZ + \
                                       (PT.IGNORE_YAW if yaw_rate_flag else PT.IGNORE_YAW_RATE)
        msg.velocity = vector_local.vector
        msg.yaw = euler_from_orientation(pose_local.pose.orientation, 'sxyz')[2]
        msg.yaw_rate = req.yaw_rate
        return position_pub, msg

    elif isinstance(req, srv.SetAttitudeRequest):
        ps.header.frame_id = req.frame_id or LOCAL_FRAME
        ps.pose.orientation = orientation_from_euler(req.roll, req.pitch, req.yaw)
        pose_local = tf_buffer.transform(ps, LOCAL_FRAME, transform_timeout)
        msg = at
        msg.orientation = pose_local.pose.orientation
        msg.thrust = req.thrust
        msg.type_mask = AT.IGNORE_YAW_RATE + AT.IGNORE_PITCH_RATE + AT.IGNORE_ROLL_RATE
        return attitude_pub, msg

    elif isinstance(req, srv.SetRatesRequest):
        msg = at
        msg.thrust = req.thrust
        msg.type_mask = AT.IGNORE_ATTITUDE
        msg.body_rate.x = req.roll_rate
        msg.body_rate.y = req.pitch_rate
        msg.body_rate.z = req.yaw_rate
        return attitude_pub, msg


current_pub = None
current_msg = None
current_req = None
current_nav_start = None
current_nav_finish = None
current_nav_start_stamp = None
handle_lock = Lock()


def handle(req):
    global current_pub, current_msg, current_req

    if not state or not state.connected:
        rospy.logwarn('No connection to the FCU')
        return {'message': 'No connection to the FCU'}

    if isinstance(req, (srv.NavigateRequest, srv.NavigateGlobalRequest)):
        if req.speed < 0:
            rospy.logwarn('Navigate speed must be positive, %s passed')
            return {'message': 'Navigate speed must be positive, %s passed' % req.speed}
        elif req.speed == 0:
            req.speed = DEFAULT_SPEED

    if isinstance(req, (srv.NavigateRequest, srv.NavigateGlobalRequest)) and \
            (pose is None or rospy.get_rostime() - pose.header.stamp > LOCAL_POSITION_TIMEOUT):
        rospy.logwarn('No local position')
        return {'message': 'No local position'}

    if getattr(req, 'yaw_rate', 0) != 0 and not math.isnan(getattr(req, 'yaw')):
        rospy.logwarn('Yaw value should be NaN for setting yaw rate')
        return {'message': 'Yaw value should be NaN for setting yaw rate'}

    if math.isnan(getattr(req, 'yaw', 0)) and math.isnan(getattr(req, 'yaw_rate', 0)):
        rospy.logwarn('Both yaw and yaw_rate cannot be NaN')
        return {'message': 'Both yaw and yaw_rate cannot be NaN'}

    try:
        # check frame_id existance
        # (for non-blocking setpoint's publishing in get_publisher_and_message)
        stamp = rospy.get_rostime()
        if hasattr(req, 'frame_id'):
            tf_buffer.lookup_transform(req.frame_id or LOCAL_FRAME, LOCAL_FRAME, stamp, TRANSFORM_TIMEOUT)

        with handle_lock:
            current_req = req
            current_pub, current_msg = get_publisher_and_message(req, stamp, False)
            rospy.loginfo('Topic: %s, message: %s', current_pub.name, current_msg)

            current_msg.header.stamp = stamp
            current_pub.publish(current_msg)

        if req.auto_arm:
            offboard_and_arm()
        else:
            if state.mode != 'OFFBOARD':
                return {'message': 'Copter is not in OFFBOARD mode, use auto_arm?'}
            if not state.armed:
                return {'message': 'Copter is not armed, use auto_arm?'}

        return {'success': True}

    except Exception as e:
        rospy.logerr(str(e))
        return {'success': False, 'message': str(e)}


def land(req):
    if not state or not state.connected:
        rospy.logwarn('No connection to the FCU')
        return {'message': 'No connection to the FCU'}

    rospy.loginfo('Set %s mode', LAND_MODE)
    res = set_mode(custom_mode=LAND_MODE)
    if not res.mode_sent:
        return {'message': 'Cannot send %s mode request' % LAND_MODE}

    start = rospy.get_rostime()
    while True:
        if state.mode == LAND_MODE:
            return {'success': True}
        if rospy.get_rostime() - start > LAND_TIMEOUT:
            return {'message': '%s mode request timed out' % LAND_MODE}
        rospy.sleep(0.1)


def release(req):
    global current_pub
    current_pub = None
    rospy.loginfo('simple_offboard: release')
    return {'success': True}


rospy.Service('navigate', srv.Navigate, handle)
rospy.Service('navigate_global', srv.NavigateGlobal, handle)
rospy.Service('set_position', srv.SetPosition, handle)
rospy.Service('set_position_global', srv.SetPositionGlobal, handle)
rospy.Service('set_velocity', srv.SetVelocity, handle)
rospy.Service('set_attitude', srv.SetAttitude, handle)
rospy.Service('set_rates', srv.SetRates, handle)
rospy.Service('land', Trigger, land)
rospy.Service('release', Trigger, release)


def get_telemetry(req):
    res = {
        'frame_id': req.frame_id or LOCAL_FRAME,
        'x': float('nan'),
        'y': float('nan'),
        'z': float('nan'),
        'lat': float('nan'),
        'lon': float('nan'),
        'alt': float('nan'),
        'vx': float('nan'),
        'vy': float('nan'),
        'vz': float('nan'),
        'pitch': float('nan'),
        'roll': float('nan'),
        'yaw': float('nan'),
        'pitch_rate': float('nan'),
        'roll_rate': float('nan'),
        'yaw_rate': float('nan'),
        'voltage': float('nan'),
        'cell_voltage': float('nan')
    }
    frame_id = req.frame_id or LOCAL_FRAME
    stamp = rospy.get_rostime()

    transform_timeout = rospy.Duration(0.4)
    try:
        if pose:
            p = tf_buffer.transform(pose, frame_id, transform_timeout)
            res['x'] = p.pose.position.x
            res['y'] = p.pose.position.y
            res['z'] = p.pose.position.z

            # Calculate roll pitch and yaw as Tait-Bryan angles, order z-y-x
            res['yaw'], res['pitch'], res['roll'] = euler_from_orientation(p.pose.orientation, axes='rzyx')
    except:
        pass

    if velocity:
        try:
            v = Vector3Stamped()
            v.header.stamp = velocity.header.stamp
            v.header.frame_id = velocity.header.frame_id
            v.vector = velocity.twist.linear
            linear = tf_buffer.transform(v, frame_id, transform_timeout)
            res['vx'] = linear.vector.x
            res['vy'] = linear.vector.y
            res['vz'] = linear.vector.z
        except:
            pass

        res['yaw_rate'] = velocity.twist.angular.z
        res['pitch_rate'] = velocity.twist.angular.y
        res['roll_rate'] = velocity.twist.angular.x

    if global_position and stamp - global_position.header.stamp < rospy.Duration(5):
        res['lat'] = global_position.latitude
        res['lon'] = global_position.longitude
        res['alt'] = global_position.altitude

    if state:
        res['connected'] = state.connected
        res['armed'] = state.armed
        res['mode'] = state.mode

    if battery:
        res['voltage'] = battery.voltage
        try:
            res['cell_voltage'] = battery.cell_voltage[0]
        except:
            pass

    return res


rospy.Service('get_telemetry', srv.GetTelemetry, get_telemetry)


rospy.loginfo('simple_offboard inited')


def start_loop():
    global current_pub, current_msg, current_req
    r = rospy.Rate(SETPOINT_RATE)

    while not rospy.is_shutdown():
        with handle_lock:
            if current_pub is not None:
                try:
                    stamp = rospy.get_rostime()

                    if getattr(current_req, 'update_frame', False) or \
                            isinstance(current_req, (srv.NavigateRequest, srv.NavigateGlobalRequest)):
                        current_pub, current_msg = get_publisher_and_message(current_req, stamp, True,
                                                                             getattr(current_req, 'update_frame', False))

                except Exception as e:
                    rospy.logwarn_throttle(10, str(e))

                current_msg.header.stamp = stamp
                current_pub.publish(current_msg)

                # For monitoring
                if isinstance(current_msg, PositionTarget):
                    p = PoseStamped()
                    p.header.frame_id = LOCAL_FRAME
                    p.header.stamp = stamp
                    p.pose.position = current_msg.position
                    p.pose.orientation = orientation_from_euler(0, 0, current_msg.yaw)
                    target_pub.publish(p)

        r.sleep()


start_loop()
