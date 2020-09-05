#!/usr/bin/env python
import rospy
import sys
import subprocess
import threading
from std_msgs.msg import String
from std_srvs.srv import Trigger
from clover_blocks.srv import Run

rospy.init_node('clover_blocks')


def run(req):
    rospy.loginfo('Run program')
    subprocess.call([sys.executable, '-c', req.code])
    print('finish')
    return {'success': True}


def stop(req):
    # TODO
    rospy.loginfo('Stop program')
    return {'message': 'Not implemented yet'}


rospy.Service('~run', Run, run)
rospy.Service('~stop', Trigger, stop)


rospy.loginfo('Ready')
rospy.spin()
