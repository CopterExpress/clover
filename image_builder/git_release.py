#!/usr/bin/env python

#
# Simple github release body-editor
# @urpylka Artem Smirnov
#
# Use:
# python git_release.py CONFIG_FILE RELEASE_ID RELEASE_BODY
#


from ConfigParser import SafeConfigParser
import requests, sys, urllib

def json_wrapper(image_name, image_link, image_size, old_text):
    # Don't need for Jenkins plugin
    #old_text = urllib.unquote_plus(old_text)
    buffer = "### Download\n* [" + image_name + ".zip](" + image_link + ") (" + image_size + ")\n\n" + old_text
    js = {}
    js["body"] = buffer
    return js


def main():

    cfgParser = SafeConfigParser()
    cfgParser.read(sys.argv[1])

    js = json_wrapper(sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[3])

    r = requests.patch(cfgParser.get('github','url') + sys.argv[2], json=js, auth=(cfgParser.get('github','login'), cfgParser.get('github','password')))

    if r.status_code == 200:
        print("Message has been successfully added!")
    else:
        return 1

if __name__ == '__main__':
    main()
