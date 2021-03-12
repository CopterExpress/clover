import threading
import time
import rospy
from clover import srv
from std_srvs.srv import Trigger
import RPi.GPIO as GPIO

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

servo1 = 33        # PWM pins
servo2 = 32

GPIO.setmode(GPIO.BOARD)    #set pin numbering system

GPIO.setup(servo1,GPIO.OUT)
GPIO.setup(servo2,GPIO.OUT)

pwm1 = GPIO.PWM(servo1,50)    #create PWM instance with frequency
pwm2 = GPIO.PWM(servo2,50)

pwm1.start(0)        #start PWM of required Duty Cycle 
pwm2.start(0)  


def servo_drop(seconds):
    print("Dropping")

    i = 1
    for num in range(seconds/2):
        if(i == 1):
            pwm1.ChangeDutyCycle(10) # right +90 deg position
            time.sleep(0.5)
            pwm1.ChangeDutyCycle(5) # left -90 deg position
            time.sleep(0.5)
            i = 2
            
        elif(i == 2):
            pwm2.ChangeDutyCycle(10) # right +90 deg position
            time.sleep(0.5)
            pwm2.ChangeDutyCycle(5) # left -90 deg position
            time.sleep(0.5)
            i = 1

        print(num)
        time.sleep(2)


if name == "main":
    # Take off and hover 2 m above the ground
    navigate(x=0, y=0, z=2, frame_id='body', auto_arm=True)

    # Wait for 5 seconds
    rospy.sleep(5)

    #dropping starts
    y = threading.Thread(target=servo_drop, args=(4,))
    y.start()

    # Fly forward 1 m
    navigate(x=2, y=0, z=0, frame_id='body')

    # Wait for 5 seconds
    rospy.sleep(5)

    # Perform landing
    land()

pwm1.stop()
pwm2.stop()
GPIO.cleanup()