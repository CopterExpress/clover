#!/usr/bin/env python3

import rospy
import functools
from clover.srv import SetLEDEffect
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDStateArray, LEDState
from util import handle_response

rospy.init_node('autotest_led', disable_signals=True)

set_leds = handle_response(rospy.ServiceProxy('led/set_leds', SetLEDs))
set_effect = handle_response(rospy.ServiceProxy('led/set_effect', SetLEDEffect))

led_count = len(rospy.wait_for_message('led/state', LEDStateArray, timeout=10).leds)
print('LED count =', led_count)

print('== Testing effects ==')

input('Fill red [enter] ')
set_effect(r=255, g=0, b=0)

input('Fill green [enter] ')
set_effect(r=0, g=100, b=0)

input('Blink white [enter] ')
set_effect(effect='blink', r=255, g=255, b=255)
rospy.sleep(3)

input('Blink fast violet [enter] ')
set_effect(effect='blink_fast', r=220, g=20, b=250)
rospy.sleep(3)

input('Fade to blue [enter] ')
set_effect(effect='fade', r=0, g=0, b=255)

input('Wipe to yellow [enter] ')
set_effect(effect='wipe', r=255, g=255, b=40)

input('Flash red [enter] ')
set_effect(effect='flash', r=255, g=0, b=0)
rospy.sleep(1)

input('Rainbow [enter] ')
set_effect(effect='rainbow')
rospy.sleep(4)

input('Rainbow fill [enter] ')
set_effect(effect='rainbow_fill')
rospy.sleep(4)

input('Turn off [enter] ')
set_effect()

print('== Testing low-level control ==')

input('Fill orange [enter] ')
set_leds(leds=[LEDState(index=i, r=245, g=155, b=0) for i in range(led_count)])

input('Fill blue gradient [enter] ')
set_leds(leds=[LEDState(index=i, r=0, g=20, b=int(255 * i / led_count)) for i in range(led_count)])

input('Animate green dot [enter] ')
set_effect()
for i in range(led_count):
    if i > 0:
        set_leds(leds=[LEDState(index=i - 1, r=0, g=0, b=0)])
    set_leds(leds=[LEDState(index=i, r=0, g=255, b=0)])
    rospy.sleep(0.05)

input('Turn off [enter] ')
set_effect()
