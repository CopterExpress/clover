#!/usr/bin/env python

# TODO: add custom header, footer
# TODO: symlinks or copy param

import subprocess
import os
import shutil
import rospy

rospy.init_node('rosstatic')

www = os.path.dirname(os.path.realpath(__file__)) + '/www'
index_file = rospy.get_param('~index', None)

shutil.rmtree(www, ignore_errors=True)  # reset www directory content
os.mkdir(www)

lines = subprocess.check_output(['rospack', 'list']).split('\n')
index = '<h1>Packages list</h1>\n<ul>\n'

for line in lines:
    if not line:
        continue
    name, path = line.split()
    if name == 'rosstatic':
        continue
    if os.path.exists(path + '/www'):
        rospy.loginfo('found www path for %s package', name)
        os.symlink(path + '/www', www + '/' + name)
        index += '<li><a href="{name}/">{name}</a></li>'.format(name=name)

if index_file is not None:
    rospy.loginfo('symlinking index file')
    os.symlink(index_file, www + '/index.html')
else:
    open(www + '/index.html', 'w').write(index)
