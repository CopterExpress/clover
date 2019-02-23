#!/usr/bin/env python

# validate all required modules installed

import rospy
from geometry_msgs.msg import PoseStamped

import cv2  # TODO - fix "NEON - NOT AVAILABLE" error
import cv2.aruco

import numpy
import mavros
from mavros_msgs.msg import State, StatusText, ExtendedState
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

from std_srvs.srv import Trigger
from clever.srv import GetTelemetry, Navigate, NavigateGlobal, SetPosition, SetVelocity, \
    SetAttitude, SetRates

import tf2_ros
import tf2_geometry_msgs

import VL53L1X
import pymavlink
from pymavlink import mavutil
import rpi_ws281x
import pigpio

print cv2.getBuildInformation()

