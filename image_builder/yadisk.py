#!/usr/bin/env python

#
# Simple python uploader to YaDisk
# Smirnov Artem @urpylka
#
# Use:
# python yadisk.py login password file server_dir
#

from YaDiskClient.YaDiskClient import YaDisk
import os.path, sys

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

            from ConfigParser import SafeConfigParser
            cfgParser = SafeConfigParser()
            cfgParser.read(sys.argv[1])
            # print "login: " + cfgParser.get('yadisk','login')
            # print "password: " + cfgParser.get('yadisk','password')
            # print "server_dir: " + cfgParser.get('yadisk','server_dir')

            upload(cfgParser.get('yadisk','login'), cfgParser.get('yadisk','password'), cfgParser.get('yadisk','server_dir'), sys.argv[2])
        else:
            print "Error: file-path or config-path is bad"
            return 1
    else:
        print "Error: amount of args is incorrect"
        return 1

if __name__ == '__main__':
    main()
