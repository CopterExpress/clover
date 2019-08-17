#!/usr/bin/env python3

import os
import sys
import subprocess

EXCLUDE = ('clever4-front-white.png', '.DS_Store', 'clever4-front-black-large.png')
code = 0

os.chdir('./docs')

for root, dirs, files in os.walk('assets'):
    for f in files:
        if f not in EXCLUDE:
            path = os.path.join(root, f)
            try:
                subprocess.check_output(['grep', '-F', '-r', path, './ru', './en'])
            except subprocess.CalledProcessError:
                print('\x1b[1;31mAssets file {} is not used\x1b[0m'.format(path))
                code = 1

# sys.exit(code) # TODO!!!
