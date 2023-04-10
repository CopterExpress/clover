import rospy
import pytest
from pytest import approx
import threading
import mavros_msgs.msg
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped
from clover import srv
from clover.msg import State
from std_srvs.srv import Trigger
from math import nan, inf
import tf2_ros
import tf2_geometry_msgs

@pytest.fixture()
def node():
    return rospy.init_node('offboard_test', anonymous=True)

@pytest.fixture
def tf_buffer():
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    return buf

def get_state():
    return rospy.wait_for_message('/simple_offboard/state', State, timeout=1)

def get_navigate_target(tf_buffer):
    target = tf_buffer.lookup_transform('map', 'navigate_target', rospy.get_rostime(), rospy.Duration(1))
    assert target.child_frame_id == 'navigate_target'
    return target

def test_offboard(node, tf_buffer):
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
    set_altitude = rospy.ServiceProxy('set_altitude', srv.SetAltitude)
    set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
    set_yaw_rate = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
    set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
    set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
    set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    land = rospy.ServiceProxy('land', Trigger)

    res = navigate()
    assert res.success == False
    assert res.message.startswith('State timeout')

    telem = get_telemetry()
    assert telem.connected == False

    # mocked state publisher
    state_pub = rospy.Publisher('/mavros/state', mavros_msgs.msg.State, latch=True, queue_size=1)
    state_msg = mavros_msgs.msg.State(mode='OFFBOARD', armed=True)

    def publish_state():
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            state_msg.header.stamp = rospy.Time.now()
            state_pub.publish(state_msg)
            r.sleep()

    # start publishing state
    threading.Thread(target=publish_state, daemon=True).start()
    rospy.sleep(0.5)

    # set_mode service mock
    def set_mode(req):
        state_msg.mode = req.custom_mode # set mocked mode to requested
        return True,

    rospy.Service('/mavros/set_mode', SetMode, set_mode)

    telem = get_telemetry()
    assert telem.connected == False

    res = navigate()
    assert res.success == False
    assert res.message.startswith('No connection to FCU')

    state_msg.connected = True
    rospy.sleep(1)

    telem = get_telemetry()
    assert telem.connected == True

    res = navigate()
    assert res.success == False
    assert res.message.startswith('No local position')

    local_position_pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, latch=True, queue_size=1)
    local_position_msg = PoseStamped()
    local_position_msg.header.frame_id = 'map'
    local_position_msg.pose.position.x = 1
    local_position_msg.pose.position.y = 2
    local_position_msg.pose.position.z = 3
    local_position_msg.pose.orientation.w = 1

    def publish_local_position():
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            local_position_msg.header.stamp = rospy.Time.now()
            local_position_pub.publish(local_position_msg)
            r.sleep()

    # start publishing local position
    threading.Thread(target=publish_local_position, daemon=True).start()
    rospy.sleep(0.5)

    # check body frame
    body = tf_buffer.lookup_transform('map', 'body', rospy.get_rostime(), rospy.Duration(1))
    assert body.child_frame_id == 'body'
    assert body.transform.translation.x == approx(1)
    assert body.transform.translation.y == approx(2)
    assert body.transform.translation.z == approx(3)

    res = navigate(x=3, y=2, z=1, frame_id='map')
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_NAVIGATE
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.x == 3
    assert state.y == 2
    assert state.z == 1
    assert state.yaw == 0
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'map'
    assert state.yaw_frame_id == 'map'
    target = get_navigate_target(tf_buffer)
    assert target.header.frame_id == 'map'
    assert target.transform.translation.x == approx(3)
    assert target.transform.translation.y == approx(2)
    assert target.transform.translation.z == approx(1)
    assert target.transform.rotation.x == 0
    assert target.transform.rotation.y == 0
    assert target.transform.rotation.z == 0
    assert target.transform.rotation.w == 1

    # try to set only the y
    res = navigate(x=nan, y=1, z=nan)
    assert res.success == False
    assert res.message.startswith('x and y can be set only together')

    # set z in body frame
    res = navigate(x=nan, y=nan, z=1, frame_id='body')
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_NAVIGATE
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.x == 3
    assert state.y == 2
    assert state.z == 4
    assert state.yaw == 0
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'map'
    assert state.yaw_frame_id == 'map'

    # set xy in test frame
    res = navigate(x=1, y=2, z=nan, frame_id='test')
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_NAVIGATE
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.x == 1
    assert state.y == 2
    assert state.z == 4
    assert state.yaw == 0
    assert state.xy_frame_id == 'test'
    assert state.z_frame_id == 'map'
    assert state.yaw_frame_id == 'test'

    # auto_arm should not invalidate the setpoint if not effective
    res = navigate(x=nan, y=nan, z=1, frame_id='map', auto_arm=True)
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_NAVIGATE
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.x == 1
    assert state.y == 2
    assert state.z == 1
    assert state.yaw == 0
    assert state.xy_frame_id == 'test'
    assert state.z_frame_id == 'map'
    assert state.yaw_frame_id == 'map'

    # auto_arm should invalidate the setpoint if effective
    state_msg.mode = 'STABILIZED' # pretend we are not in OFFBOARD mode
    rospy.sleep(1)
    res = navigate(x=nan, y=nan, z=1, frame_id='map', auto_arm=True)
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_NAVIGATE
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.x == 1
    assert state.y == 2
    assert state.z == 1
    assert state.yaw == 0
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'map'
    assert state.yaw_frame_id == 'map'
    state_msg.mode = 'OFFBOARD'
    rospy.sleep(1)

    # set_attitude should invalidate the setpoint
    res = set_attitude()
    assert res.success == True

    res = navigate(x=5, y=6, z=nan, yaw=nan, frame_id='map')
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_NAVIGATE
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.x == 5
    assert state.y == 6
    assert state.z == 3
    assert state.yaw == 0
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'map'
    assert state.yaw_frame_id == 'map'

    # test set_altitude
    res = set_altitude(z=7, frame_id='test')
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_NAVIGATE
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.x == 5
    assert state.y == 6
    assert state.z == 7
    assert state.yaw == 0
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'test'
    assert state.yaw_frame_id == 'map'

    # test set_yaw
    res = set_yaw(yaw=0.5, frame_id='test2')
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_NAVIGATE
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.x == 5
    assert state.y == 6
    assert state.z == 7
    assert state.yaw == 0.5
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'test'
    assert state.yaw_frame_id == 'test2'

    # test set_yaw_rate
    res = set_yaw_rate(yaw_rate=2)
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_NAVIGATE
    assert state.yaw_mode == State.YAW_MODE_YAW_RATE
    assert state.x == 5
    assert state.y == 6
    assert state.z == 7
    assert state.yaw_rate == 2
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'test'

    # navigate(yaw=nan) should keep yaw rate mode
    res = navigate(x=nan, y=nan, z=nan, yaw=nan)
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_NAVIGATE
    assert state.yaw_mode == State.YAW_MODE_YAW_RATE
    assert state.x == 5
    assert state.y == 6
    assert state.z == 7
    assert state.yaw_rate == 2
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'test'

    # set_yaw(nan) should change back to yaw mode
    res = set_yaw(yaw=nan)
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_NAVIGATE
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.yaw == 0
    assert state.yaw_frame_id == 'map'

    # test set_position
    res = set_position(x=nan, y=nan, z=13, yaw=nan, frame_id='test2')
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_POSITION
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.x == 5
    assert state.y == 6
    assert state.z == 13
    assert state.yaw == 0
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'test2'
    assert state.yaw_frame_id == 'map'

    # set_altitude should not change the mode
    res = set_altitude(z=3, frame_id='test')
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_POSITION
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.x == 5
    assert state.y == 6
    assert state.z == 3
    assert state.yaw == 0
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'test'
    assert state.yaw_frame_id == 'map'

    # set_yaw should not change the main mode
    res = set_yaw(yaw=1, frame_id='test2')
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_POSITION
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.x == 5
    assert state.y == 6
    assert state.z == 3
    assert state.yaw == 1
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'test'
    assert state.yaw_frame_id == 'test2'

    # test set_velocity
    res = set_velocity(vx=1, frame_id='body')
    state = get_state()
    assert state.mode == State.MODE_VELOCITY
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.vx == 1
    assert state.vy == 0
    assert state.vz == 0
    assert state.yaw == 0
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'map'
    assert state.yaw_frame_id == 'map'

    # set_altitude should not work in velocity mode
    res = set_altitude(z=3, frame_id='test')
    assert res.success == False
    assert res.message.startswith('Altitude cannot be set in')

    # test set_attitude
    res = set_attitude(roll=0.1, pitch=0.2, yaw=0.3, thrust=0.5)
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_ATTITUDE
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.roll == approx(0.1)
    assert state.pitch == approx(0.2)
    assert state.yaw == approx(0.3)
    assert state.thrust == approx(0.5)
    assert state.yaw_frame_id == 'map'
    msg = rospy.wait_for_message('/mavros/setpoint_attitude/attitude', PoseStamped, timeout=3)
    # Tait-Bryan ZYX angle (rzyx) converted to quaternion
    assert msg.pose.orientation.x == approx(0.0342708)
    assert msg.pose.orientation.y == approx(0.10602051)
    assert msg.pose.orientation.z == approx(0.14357218)
    assert msg.pose.orientation.w == approx(0.98334744)
    msg = rospy.wait_for_message('/mavros/setpoint_attitude/thrust', mavros_msgs.msg.Thrust, timeout=3)
    assert msg.thrust == approx(0.5)

    # set_yaw should work in attitude mode
    res = set_yaw(yaw=0.7, frame_id='test2')
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_ATTITUDE
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.roll == approx(0.1)
    assert state.pitch == approx(0.2)
    assert state.yaw == approx(0.7)
    assert state.thrust == approx(0.5)
    assert state.yaw_frame_id == 'test2'

    # set_yaw_rate should not work in attitude mode
    res = set_yaw_rate(yaw_rate=0.3)
    assert res.success == False
    assert res.message.startswith('Yaw rate cannot be set in')

    # test set_rates
    res = set_rates(roll_rate=nan, pitch_rate=nan, yaw_rate=0.3, thrust=0.6)
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_RATES
    assert state.yaw_mode == State.YAW_MODE_YAW_RATE
    assert state.roll_rate == approx(0)
    assert state.pitch_rate == approx(0)
    assert state.yaw_rate == approx(0.3)
    assert state.thrust == approx(0.6)
    msg = rospy.wait_for_message('/mavros/setpoint_raw/attitude', mavros_msgs.msg.AttitudeTarget, timeout=3)
    assert msg.thrust == approx(0.6)

    res = set_rates(roll_rate=0.3, pitch_rate=0.2, yaw_rate=0.1, thrust=0.4)
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_RATES
    assert state.yaw_mode == State.YAW_MODE_YAW_RATE
    assert state.roll_rate == approx(0.3)
    assert state.pitch_rate == approx(0.2)
    assert state.yaw_rate == approx(0.1)
    assert state.thrust == approx(0.4)

    res = set_rates(roll_rate=nan, pitch_rate=nan, yaw_rate=nan, thrust=0.3)
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_RATES
    assert state.yaw_mode == State.YAW_MODE_YAW_RATE
    assert state.roll_rate == approx(0.3)
    assert state.pitch_rate == approx(0.2)
    assert state.yaw_rate == approx(0.1)
    assert state.thrust == approx(0.3)
    msg = rospy.wait_for_message('/mavros/setpoint_raw/attitude', mavros_msgs.msg.AttitudeTarget, timeout=3)
    assert msg.type_mask == mavros_msgs.msg.AttitudeTarget.IGNORE_ATTITUDE
    assert msg.body_rate.x == approx(0.3)
    assert msg.body_rate.y == approx(0.2)
    assert msg.body_rate.z == approx(0.1)

    # set_yaw_rate should work in rates mode
    res = set_yaw_rate(yaw_rate=0.4)
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_RATES
    assert state.yaw_mode == State.YAW_MODE_YAW_RATE
    assert state.roll_rate == approx(0.3)
    assert state.pitch_rate == approx(0.2)
    assert state.yaw_rate == approx(0.4)
    assert state.thrust == approx(0.3)

    res = set_rates(roll_rate=inf)
    assert res.success == False
    assert res.message == 'roll_rate argument cannot be Inf'

    # test land service
    res = land()
    assert res.success == True
    assert state_msg.mode == 'AUTO.LAND' # check that the mode was set correctly
