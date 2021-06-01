#!/usr/bin/env python3

# validate all required modules installed

import rospy
from geometry_msgs.msg import PoseStamped

import cv2
import cv2.aruco

import numpy
import mavros
from mavros_msgs.msg import State, StatusText, ExtendedState
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

from std_srvs.srv import Trigger
from clover.srv import GetTelemetry, Navigate, NavigateGlobal, SetPosition, SetVelocity, \
    SetAttitude, SetRates, SetLEDEffect

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

from . import qr
