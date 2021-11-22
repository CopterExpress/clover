#!/usr/bin/env python3

import pigpio
pi = pigpio.pi()
pi.set_mode(24, pigpio.OUTPUT)
pi.write(24, True)
