#!/usr/bin/env python

import rospy
from flask import Flask, send_from_directory, send_file


rospy.init_node('web_server', disable_signals=True)


port = rospy.get_param('~port', 7070)
host = rospy.get_param('~host', '0.0.0.0')
serve_path = rospy.get_param('~path')
app = Flask(__name__)


@app.route('/')
def serve_index():
    return send_from_directory(serve_path, 'index.html')


@app.route('/<path:path>')
def serve_static(path):
    print serve_path, path
    return send_from_directory(serve_path, path)


rospy.loginfo('Serving on %s:%s', host, port)
app.run(host=host, port=port, threaded=True)
