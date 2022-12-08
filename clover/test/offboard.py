import rospy
import pytest
from pytest import approx
import threading
import mavros_msgs.msg
from geometry_msgs.msg import PoseStamped
from clover import srv
from clover.msg import State
from math import nan

@pytest.fixture()
def node():
    return rospy.init_node('offboard_test', anonymous=True)

def get_state():
    return rospy.wait_for_message('/simple_offboard/state', State, timeout=1)

def test_offboard(node):
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
    set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    res = navigate()
    assert res.success == False
    assert res.message.startswith('State timeout')

    telem = get_telemetry()
    assert telem.connected == False

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

    res = navigate(z=1, frame_id='map')
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_NAVIGATE
    assert state.yaw_mode == State.YAW_MODE_YAW
    assert state.z == 1
    assert state.yaw == 0
    assert state.xy_frame_id == 'map'
    assert state.z_frame_id == 'map'
    assert state.yaw_frame_id == 'map'

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

    # test set_rates
    res = set_rates(roll_rate=nan, pitch_rate=nan, yaw_rate=0.3, thrust=0.5)
    assert res.success == True
    state = get_state()
    assert state.mode == State.MODE_RATES
    assert state.yaw_mode == State.YAW_MODE_YAW_RATE
    assert state.roll_rate == approx(0)
    assert state.pitch_rate == approx(0)
    assert state.yaw_rate == approx(0.3)
    assert state.thrust == approx(0.5)

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
