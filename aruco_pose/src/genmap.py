#!/usr/bin/env python

# Copyright (C) 2018 Copter Express Technologies
#
# Author: Oleg Kalachev <okalachev@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

"""Markers map generator

Generate map file for aruco_map nodelet.

Usage:
  genmap.py <length> <x> <y> <dist_x> <dist_y> [<first>] [<x0>] [<y0>] [--top-left | --bottom-left] [-o <filename>]
  genmap.py (-h | --help)

Options:
  <length>       Marker side length
  <x>            Marker count along X axis
  <y>            Marker count along Y axis
  <dist_x>       Distance between markers along X axis
  <dist_y>       Distance between markers along Y axis
  <first>        First marker ID [default: 0]
  <x0>           X coordinate for the first marker [default: 0]
  <y0>           Y coordinate for the first marker [default: 0]
  --top-left     First marker is on top-left (default)
  --bottom-left  First marker is on bottom-left
  -o <filename>  Output map file name in the 'map' subdirectory of aruco_pose package

Example:
  rosrun aruco_pose genmap.py 0.33 2 4 1 1 0 -o test_map.txt
"""

from __future__ import print_function

import sys
from os import path
from docopt import docopt


arguments = docopt(__doc__)

length = float(arguments['<length>'])
first = int(arguments['<first>'] if arguments['<first>'] is not None else 0)
x0 = float(arguments['<x0>'] if arguments['<x0>'] is not None else 0)
y0 = float(arguments['<y0>'] if arguments['<y0>'] is not None else 0)
markers_x = int(arguments['<x>'])
markers_y = int(arguments['<y>'])
dist_x = float(arguments['<dist_x>'])
dist_y = float(arguments['<dist_y>'])
bottom_left = arguments['--bottom-left']

if arguments['-o'] is None:
    output = sys.stdout
else:
    output = open(path.join(path.dirname(__file__), '..', 'map', arguments['-o']), 'w')

max_y = y0 + (markers_y - 1) * dist_y

output.write('# id\tlength\tx\ty\tz\trot_z\trot_y\trot_x\n')
for y in range(markers_y):
    for x in range(markers_x):
        pos_x = x0 + x * dist_x
        pos_y = y0 + y * dist_y
        if not bottom_left:
            pos_y = max_y - pos_y
        output.write('{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n'.format(first, length, pos_x, pos_y, 0, 0, 0, 0))
        first += 1
