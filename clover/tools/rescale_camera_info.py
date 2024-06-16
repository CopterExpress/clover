#!/usr/bin/env python3

# Copyright (C) 2024 Copter Express Technologies
#
# Author: Oleg Kalachev <okalachev@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

"""Rescale camera info

Rescale camera info files for different resolutions.

Usage:
  rescale_camera_info.py <camera_info_file>
  rescale_camera_info.py (-h | --help)

Options:
  <camera_info_file>  Path to the source camera info file

Example:
  rescale_camera_info.py camera_info.yaml
"""

from docopt import docopt
import yaml

arguments = docopt(__doc__)

camera_info = yaml.safe_load(open(arguments['<camera_info_file>']))
RESOLUTIONS = (
    (320, 240), # QVGA
    (640, 480), # VGA
    (800, 600), # SVGA
    (1280, 720), # HD
    (1920, 1080), # FullHD
    (2592, 1944), # 5MP
    (3840, 2160), # 4K
    (4056, 3040),
)
# TODO: retrieve resolutions list (v4l2-ctl --list-formats-ext)

for resolution in RESOLUTIONS:
    width_k = resolution[0] / camera_info['image_width']
    height_k = resolution[1] / camera_info['image_height']

    camera_info_rescaled = camera_info.copy()
    camera_info_rescaled['image_width'] = resolution[0]
    camera_info_rescaled['image_height'] = resolution[1]

    # See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
    camera_info_rescaled['camera_matrix']['data'][0] *= width_k
    camera_info_rescaled['camera_matrix']['data'][2] *= width_k
    camera_info_rescaled['camera_matrix']['data'][4] *= height_k
    camera_info_rescaled['camera_matrix']['data'][5] *= height_k

    camera_info_rescaled['projection_matrix']['data'][0] *= width_k
    camera_info_rescaled['projection_matrix']['data'][2] *= width_k
    camera_info_rescaled['projection_matrix']['data'][5] *= height_k
    camera_info_rescaled['projection_matrix']['data'][6] *= height_k

    output_file = arguments['<camera_info_file>'].replace('.yaml', '_{}x{}.yaml'.format(resolution[0], resolution[1]))
    with open(output_file, 'w') as f:
        f.write('# Generated from {} by rescale_camera_info.py\n'.format(arguments['<camera_info_file>']))
        yaml.dump(camera_info_rescaled, f)

    print('Saved {}'.format(output_file))
