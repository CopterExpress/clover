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

#* incialização
rospy.init_node('roswww_static')

rospack = rospkg.RosPack()

www = rospkg.get_ros_home() + '/www'
index_file = rospy.get_param('~index_file', None)
default_package = rospy.get_param('~default_package', None)

shutil.rmtree(www, ignore_errors=True)  # reset www directory content
os.mkdir(www)

packages = rospack.list()

#* procura dos pacotes
names = list()
others = list()
for name in packages:
    path = rospack.get_path(name)
    if os.path.exists(path + '/www'):
        rospy.loginfo('found www path for %s package', name)
        os.symlink(path + '/www', www + '/' + name)
        if name == 'clover' or name == 'clover_blocks' or name == 'swarm_clover_blocks':
            continue
        else:
            others.append(name)

#* index.html (e bottom.html)

# cabeçalho
f = open('index.txt', 'r')
index = ''
for line in f:
    index += str(line)

# caso em que há outros packages além dos 3 padrões (sem formatação)
if len(others) > 0:
    index += '<br><h1>Other Packages Avaiable</h1>\n'
    for others in others:
        index += '<br><a href="{name}/">{name}</a>\n'.format(name=other)

# finaliza o index
index += '\n</body>\n</html>'
f.close()

# bottom
f = open('bottom.txt', 'r')
bottom = ''
for line in f:
    bottom += str(line)
f.close()


#* style.css

f = open('style.txt', 'r')
style = ''
for line in f:
    style += str(line)
f.close()

#* script.js

f = open('script.txt', 'r')
script = ''
for line in f:
    script += str(line)
f.close()


#* Escreve nos arquivos 

#* index.html
if default_package is not None:
    redirect_html = '<meta http-equiv=refresh content="0; url={name}/">'.format(name=default_package)
    open(www + '/index.html', 'w').write(redirect_html)
elif index_file is not None:
    rospy.loginfo('symlinking index file')
    os.symlink(index_file, www + '/index.html')
else:
    open(www + '/index.html', 'w').write(index)

#* bottom.html
if default_package is not None:
    redirect_html = '<meta http-equiv=refresh content="0; url={name}/">'.format(name=default_package)
    open(www + '/bottom.html', 'w').write(redirect_html)
elif index_file is not None:
    rospy.loginfo('symlinking bottom file')
    os.symlink(index_file, www + '/bottom.html')
else:
    open(www + '/bottom.html', 'w').write(bottom)

#* style.css
if default_package is not None:
    redirect_html = '<meta http-equiv=refresh content="0; url={name}/">'.format(name=default_package)
    open(www + '/style.css', 'w').write(redirect_html)
elif index_file is not None:
    rospy.loginfo('symlinking style file')
    os.symlink(index_file, www + '/style.html')
else:
    open(www + '/style.css', 'w').write(style)

#* script.js
if default_package is not None:
    redirect_html = '<meta http-equiv=refresh content="0; url={name}/">'.format(name=default_package)
    open(www + '/script.js', 'w').write(redirect_html)
elif index_file is not None:
    rospy.loginfo('symlinking script file')
    os.symlink(index_file, www + '/script.js')
else:
    open(www + '/script.js', 'w').write(script)

#* copy assests folder
src = os.getcwd() + '/assets/'
dst = rospkg.get_ros_home() + '/www/assets'
os.mkdir(dst)
try:
    if os.path.exists(dst):
        shutil.rmtree(dst)
        shutil.copytree(src, dst)
except OSError as e:
    if e.errno == errno.ENOTDIR:
        shutil.copy(source_dir_prompt, destination_dir_prompt)
    else:
        print('Directory not copied. Error: %s' % e)