#!/usr/bin/env python

import rospy
import os, sys
import subprocess
import threading
import signal
import re
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from clover_blocks.srv import Run, Load, Store

rospy.init_node('clover_blocks')
process = None
running_lock = threading.Lock()
running_pub = rospy.Publisher('~running', Bool, queue_size=1, latch=True)
running_pub.publish(False)


def run(req):
    if not running_lock.acquire(False):
        return {'message': 'Already running'}

    try:
        global process
        rospy.loginfo('Run program')
        running_pub.publish(True)
        process = subprocess.Popen([sys.executable, '-c', req.code])

        def wait_thread():
            process.wait()
            rospy.loginfo('Program terminated')
            running_lock.release()
            running_pub.publish(False)

        t = threading.Thread(target=wait_thread)
        t.start()

        return {'success': True}

    except Exception as e:
        running_lock.release()
        return {'message': str(e)}


def stop(req):
    global process
    if process:
        rospy.loginfo('Stop program')
        process.send_signal(signal.SIGINT)
        process = None
        return {'success': True}
    else:
        return {'success': True, 'message': 'Program not running'}


programs_path = rospy.get_param('~programs_dir', os.path.dirname(os.path.abspath(__file__)) + '/../programs')


def load(req):
    res = {'names': [], 'programs': [], 'success': True}
    programs = []
    try:
        for currentpath, folders, files in os.walk(programs_path):
            for f in files:
                filename = os.path.join(currentpath, f)
                res['names'].append(os.path.relpath(filename, programs_path))
                res['programs'].append(open(filename, 'r').read())
        return res
    except Exception as e:
        rospy.logerr(e)
        return {'names': [], 'programs': [], 'message': str(e)}


name_regexp = re.compile(r'^[a-zA-Z-_.]{0,20}$')

def store(req):
    if not name_regexp.match(req.name):
        return {'message': 'Bad program name'}

    filename = os.path.abspath(os.path.join(programs_path, req.name))
    try:
        open(filename, 'w').write(req.program)
        return {'success': True, 'message': 'Stored to ' + filename}
    except Exception as e:
        rospy.logerr(e)
        return {'names': [], 'programs': [], 'message': str(e)}


rospy.Service('~run', Run, run)
rospy.Service('~stop', Trigger, stop)
rospy.Service('~load', Load, load)
rospy.Service('~store', Store, store)


rospy.loginfo('Ready')
rospy.spin()
