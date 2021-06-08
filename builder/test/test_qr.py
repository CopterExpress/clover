#!/usr/bin/env python3

# Test QG recognition example
# Should be synced with the documentation: /docs/en/camera.md, /docs/ru/camera.md
# TODO: use real ROS topics

import rospy
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()

# rospy.init_node('barcode_test')

# Image subscriber callback function
def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        b_data = barcode.data.decode("utf-8")
        b_type = barcode.type
        (x, y, w, h) = barcode.rect
        xc = x + w/2
        yc = y + h/2
        print("Found {} with data {} with center at x={}, y={}".format(b_type, b_data, xc, yc))
    # rospy.signal_shutdown('done')

# image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)

# ==============================================================================
# Publish test image
# rospy.sleep(2)

import cv2
img = cv2.imread('qr.png')
image_callback(bridge.cv2_to_imgmsg(img, 'bgr8'))

# image_pub = rospy.Publisher('/main_camera/image_raw', Image, queue_size=1, latch=True)
# image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

# rospy.spin()
