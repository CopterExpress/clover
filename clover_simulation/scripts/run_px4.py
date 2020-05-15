#!/usr/bin/env python

import os
import catkin_pkg
import rospy

rospy.init_node('px4_runner', anonymous=True)

rospy.logdebug('Current environment: {}'.format(os.environ))

rospy.loginfo('PX4 sim model: {}'.format(os.environ['PX4_SIM_MODEL']))
rospy.loginfo('PX4 estimator: {}'.format(os.environ['PX4_ESTIMATOR']))

px4_source_path = rospy.get_param('~px4_source_path', None)
ID = rospy.get_param('~ID', 0)

if px4_source_path is None:
    rospy.logerr('px4_source_path is not set. Set it to your PX4 source checkout')
    exit(1)

if not os.path.exists('{}/ROMFS'.format(px4_source_path)):
    rospy.logerr('Could not find ROMFS in {}, set px4_source_path to your PX4 source checkout'.format(px4_source_path))
    exit(1)

rospy.loginfo('Looking for PX4 binary in {}'.format(px4_source_path))

px4_binary_path = None

for root, dirs, files in os.walk(px4_source_path):
    if 'px4' in files:
        px4_binary_path = os.path.join(root, 'px4')
        break

if px4_binary_path is None:
    rospy.logerr('Could not find built PX4 binary, run `make px4_sitl` in {}'.format(px4_source_path))

rospy.loginfo('PX4 binary path: {}'.format(px4_binary_path))

os.execv(px4_binary_path, [px4_binary_path, '-w', px4_source_path, '{}/ROMFS/px4fmu_common'.format(px4_source_path), '-s', 'etc/init.d-posix/rcS', '-i', str(ID)])
