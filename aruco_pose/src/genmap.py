#!/usr/bin/env python

"""Markers map generator

Generate map file for aruco_map nodelet.

Usage:
  genmap.py <length> <x> <y> <dist_x> <dist_y> <first> [--top-left]
  genmap.py (-h | --help)

Options:
  <length>    Marker side length
  <x>         Marker count along X axis
  <y>         Marker count along Y axis
  <dist_x>    Distance between markers along X axis
  <dist_y>    Distance between markers along Y axis
  <sep_x>     Space beetween markers along X axis
  <sep_y>     Space beetween markers along Y axis
  <first>     First marker ID
  --top-left  First marker is on top-left (not bottom-left)
"""

from __future__ import print_function

from docopt import docopt


arguments = docopt(__doc__)

length = float(arguments['<length>'])
first = int(arguments['<first>'])
markers_x = int(arguments['<x>'])
markers_y = int(arguments['<y>'])
dist_x = float(arguments['<dist_x>'])
dist_y = float(arguments['<dist_y>'])
top_left = arguments['--top-left']

max_y = markers_y * length

for y in range(markers_y):
    for x in range(markers_x):
        pos_x = x * dist_x
        pos_y = y * dist_y
        if top_left:
            pos_y = max_y - pos_y
        print('{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t'.format(first, length, pos_x, pos_y, 0, 0, 0, 0))
        first += 1
