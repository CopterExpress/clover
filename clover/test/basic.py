#!/usr/bin/env python
import rospy
import pytest
from mavros_msgs.msg import State
from clover import srv
import time

@pytest.fixture()
def node():
    return rospy.init_node('clover_test', anonymous=True)

def test_state(node):
    state = rospy.wait_for_message('mavros/state', State, timeout=10)
    assert state.connected == False
    assert state.armed == False
    assert state.guided == False
    assert state.mode == ''

def test_simple_offboard_services_available():
    rospy.wait_for_service('get_telemetry', timeout=5)
    rospy.wait_for_service('navigate', timeout=5)
    rospy.wait_for_service('navigate_global', timeout=5)
    rospy.wait_for_service('set_position', timeout=5)
    rospy.wait_for_service('set_velocity', timeout=5)
    rospy.wait_for_service('set_attitude', timeout=5)
    rospy.wait_for_service('set_rates', timeout=5)
    rospy.wait_for_service('land', timeout=5)
    rospy.wait_for_service('simple_offboard/release', timeout=5)

def test_web_video_server(node):
    try:
        # Python 2
        import urllib2 as urllib
    except ModuleNotFoundError:
        # Python 3
        import urllib.request as urllib
    urllib.urlopen("http://localhost:8080").read()

def test_blocks(node):
    rospy.wait_for_service('clover_blocks/run', timeout=5)
    rospy.wait_for_service('clover_blocks/stop', timeout=5)
    rospy.wait_for_service('clover_blocks/load', timeout=5)
    rospy.wait_for_service('clover_blocks/store', timeout=5)

    from std_msgs.msg import String
    from clover_blocks.srv import Run

    def wait_print():
        try:
            wait_print.result = rospy.wait_for_message('clover_blocks/print', String, timeout=5).data
        except Exception as e:
            wait_print.result = str(e)

    import threading
    t = threading.Thread(target=wait_print)
    t.start()
    rospy.sleep(0.1)

    run = rospy.ServiceProxy('clover_blocks/run', Run)
    assert run(code='print("test")').success == True

    t.join()
    assert wait_print.result == 'test'

def test_long_callback():
    from clover import long_callback
    from time import sleep

    # very basic test for long_callback
    @long_callback
    def cb(i):
        cb.counter += i
    cb.counter = 0
    cb(2)
    sleep(0.1)
    cb(3)
    sleep(1)
    assert cb.counter == 5
