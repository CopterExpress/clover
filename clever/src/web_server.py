#!/usr/bin/env python

import rospy
import subprocess
import re
from flask import Flask, send_from_directory, send_file, request, jsonify

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


@app.route('/wifi_data/')
def get_wifi_data():
    cur_ip = request.remote_addr
    ip_signal = get_ip_signal()
    return jsonify({'ip': cur_ip, 'signal': ip_signal[cur_ip]}), 200


def get_ip_signal():
    wlan_interface = 'wlan0'
    # Getting info about wifi client connected to access point. From here we know MAC and signal level
    iwl = subprocess.check_output(['sudo', 'iw', 'dev', 'wlan0', 'station', 'dump']).splitlines()
    mac_signal = {}
    cur_client = ''
    for line in iwl:
        if line.find('Station') != -1:
            cur_client = re.search(r'([0-9A-F]{2}[:-]){5}([0-9A-F]{2})', line, re.I).group()
        if line.find('signal') != -1:
            sg = re.search(r'(\[-?\d*\])', line, re.I).group()
            mac_signal[cur_client] = re.sub(r'[\[\]]', '', sg)
    ip_signal = {}
    # Getting ip-mac mapping
    ip_mac = subprocess.check_output(['arp', '-i', wlan_interface]).splitlines()
    for line in ip_mac:
        mac = re.search(r'([0-9A-F]{2}[:-]){5}([0-9A-F]{2})', line, re.I)
        if mac is not None:
            mac = mac.group()
            if mac in mac_signal:
                ips = re.search(r'((2[0-5]|1[0-9]|[0-9])?[0-9]\.){3}((2[0-5]|1[0-9]|[0-9])?[0-9])', line, re.I).group()
                ip_signal[ips] = mac_signal[mac]
    return ip_signal


rospy.loginfo('Serving on %s:%s', host, port)
app.run(host=host, port=port, threaded=True)
