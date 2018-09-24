#!/usr/bin/env python

#
# The simple python uploader to YaDisk
# Use: python yadisk.py login password file server_dir
#
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Artem Smirnov <urpylka@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

from YaDiskClient.YaDiskClient import YaDisk
import os.path, sys, json

def upload(_login, _password, _server_dir, _file):
    if os.path.isfile(_file):
        disk = YaDisk(_login, _password)
        disk.upload(_file, _server_dir + '/' + os.path.basename(_file))
        link = disk.publish_doc(_server_dir + '/' + os.path.basename(_file))
        print link
    else:
        print "Error: file-path is bad"
        return 1

def main():
    if (len(sys.argv) == 5):
        print "login: " + sys.argv[1]
        print "password: " + sys.argv[2]
        print "server_dir: " + sys.argv[3]
        print "file: " + sys.argv[4]
        
        upload(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])

    elif (len(sys.argv) == 3):
    	# print "config: " + sys.argv[1]
        # print "file: " + sys.argv[2]
        
        if os.path.isfile(sys.argv[1]) and os.path.isfile(sys.argv[2]):

            with open(sys.argv[1]) as json_data:
                d = json.load(json_data)
                upload(d['yadisk']['login'], d['yadisk']['password'], d['yadisk']['server_dir'], sys.argv[2])

        else:
            print "Error: file-path or config-path is bad"
            return 1
    else:
        print "Error: amount of args is incorrect"
        return 1

if __name__ == '__main__':
    main()
