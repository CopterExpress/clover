#!/usr/bin/env python

# Copyright (C) 2020 Copter Express Technologies
#
# Author: Oleg Kalachev <okalachev@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# TODO: add custom header, footer
# TODO: symlinks or copy param

import os
import shutil
import rospy
import rospkg

rospy.init_node('roswww_static')

rospack = rospkg.RosPack()

www = rospkg.get_ros_home() + '/www'
index_file = rospy.get_param('~index_file', None)
default_package = rospy.get_param('~default_package', None)

shutil.rmtree(www, ignore_errors=True)  # reset www directory content
os.mkdir(www)

packages = rospack.list()

index = '<h1>Packages list</h1>\n<ul>\n'

for name in packages:
    path = rospack.get_path(name)
    if os.path.exists(path + '/www'):
        rospy.loginfo('found www path for %s package', name)
        os.symlink(path + '/www', www + '/' + name)
        index += '<li><a href="{name}/">{name}</a></li>'.format(name=name)

if default_package is not None:
    redirect_html = '<meta http-equiv=refresh content="0; url={name}/">'.format(name=default_package)
    open(www + '/index.html', 'w').write(redirect_html)
elif index_file is not None:
    rospy.loginfo('symlinking index file')
    os.symlink(index_file, www + '/index.html')
else:
    open(www + '/index.html', 'w').write(index)
