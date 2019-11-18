#!/usr/bin/env python3

import os
import sys

def human_size(num, suffix='B'):
    for unit in ['', 'Ki', 'Mi', 'Gi', 'Ti', 'Pi', 'Ei', 'Zi']:
        if abs(num) < 1024.0:
            return "%3.1f %s%s" % (num, unit, suffix)
        num /= 1024.0
    return "%.1f %s%s" % (num, 'Yi', suffix)

SIZE_LIMIT = 800 * 1024
EXCLUDE = 'rviz.png', 'ssid.png', 'sitl_docker_demo.png', 'qgc-params.png', 'butterfly.png', \
    'Clever main.png', 'fpv_3.png', '1_4.png', 'fpv_4.png', 'detal1.png', 'lockradio.png', \
    'qground.png', 'allElements.png', 'download-log.png', 'explosion.png', 'rqt.png', \
    'cl3_mountBEC.JPG', 'cl3_mountRpiCamera.JPG', 'clever4-front-black-large.png', \
    'qgc-battery.png', 'qgc-radio.png', 'qgc-cal-acc.png', 'qgc-esc.png', 'qgc-cal-compass.png', \
    'qgc.png', 'qgc-parameters.png', 'clever4-front-white-large.png', 'qgc-modes.png', \
    'qgc-requires-setup.png', 'clever4-front-white.png', 'clever4-kit-white.png', '26_1.png', 'battery_holder.stl'

code = 0

for root, dirs, files in os.walk('docs/'):
    for f in files:
        if f not in EXCLUDE:
            path = os.path.join(root, f)
            size = os.path.getsize(path)
            if size > SIZE_LIMIT:
                print('\x1b[1;31mFile too large ({}): {}\x1b[0m'.format(human_size(size), path), \
                    file=sys.stderr)
                code = 1

sys.exit(code)
