#!/usr/bin/env python3

# validate all required modules installed

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range, BatteryState

import cv2
import cv2.aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy
import mavros
from mavros_msgs.msg import State, StatusText, ExtendedState, RCIn, Mavlink
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

from std_srvs.srv import Trigger
from clover.srv import GetTelemetry, Navigate, NavigateGlobal, SetPosition, SetVelocity, \
    SetAttitude, SetRates, SetLEDEffect
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDStateArray, LEDState
from aruco_pose.msg import Marker, MarkerArray, Point2D

import dynamic_reconfigure.client

import tf2_ros
import tf2_geometry_msgs

import VL53L1X
import pymavlink
from pymavlink import mavutil
import rpi_ws281x
import pigpio
# from espeak import espeak
from pyzbar import pyzbar

print(cv2.getBuildInformation())
