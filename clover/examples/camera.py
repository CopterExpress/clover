# Information: https://clover.coex.tech/camera

# Example on basic working with the camera and image processing:

# - cuts out a central square from the camera image;
# - publishes this cropped image to the topic `/cv/center`;
# - computes the average color of it;
# - prints its name to the console.

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback

rospy.init_node('cv')
bridge = CvBridge()

printed_color = None
center_pub = rospy.Publisher('~center', Image, queue_size=1)

def get_color_name(h):
    if h < 15: return 'red'
    elif h < 30: return 'orange'
    elif h < 60: return 'yellow'
    elif h < 90: return 'green'
    elif h < 120: return 'cyan'
    elif h < 150: return 'blue'
    elif h < 170: return 'magenta'
    else: return 'red'


@long_callback
def image_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')

    # convert to HSV to work with color hue
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # cut out a central square
    w = img.shape[1]
    h = img.shape[0]
    r = 20
    center = img_hsv[h // 2 - r:h // 2 + r, w // 2 - r:w // 2 + r]

    # compute and print the average hue
    mean_hue = center[:, :, 0].mean()
    color = get_color_name(mean_hue)
    global printed_color
    if color != printed_color:
        print(color)
        printed_color = color

    # publish the cropped image
    center = cv2.cvtColor(center, cv2.COLOR_HSV2BGR)
    center_pub.publish(bridge.cv2_to_imgmsg(center, 'bgr8'))

# process every frame:
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)

# process 5 frames per second:
# image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)

rospy.spin()
