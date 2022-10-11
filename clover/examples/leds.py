# Information: https://clover.coex.tech/led

import rospy
from clover.srv import SetLEDEffect

rospy.init_node('leds')

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service

print('Fill red')
set_effect(r=255, g=0, b=0)  # fill strip with red color
rospy.sleep(2)

print('Fill green')
set_effect(r=0, g=100, b=0)  # fill strip with green color
rospy.sleep(2)

print('Fade to blue')
set_effect(effect='fade', r=0, g=0, b=255)  # fade to blue color
rospy.sleep(2)

print('Flash red')
set_effect(effect='flash', r=255, g=0, b=0)  # flash twice with red color
rospy.sleep(2)

print('Blink white')
set_effect(effect='blink', r=255, g=255, b=255)  # blink with white color
rospy.sleep(5)

print('Rainbow')
set_effect(effect='rainbow')  # show rainbow
