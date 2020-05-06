#!/usr/bin/env python
import rospy
import pytest
from mavros_msgs.msg import State
from clover import srv

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

def test_web_video_server(node):
    import urllib2
    urllib2.urlopen("http://localhost:8080").read()

def test_shell(node):
    execute = rospy.ServiceProxy('exec', srv.Execute)
    execute.wait_for_service(5)

    res = execute(cmd='echo foo')
    assert res.code == 0
    assert res.output == 'foo\n'

    res = execute(cmd='foo')
    assert res.code == 32512
    assert res.output == ''

    res = execute(cmd='ls foo')
    assert res.code == 512
    assert res.output == ''
