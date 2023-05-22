# This example makes the drone find and follow the red circle.
# To test in the simulator, place 'Red Circle' model on the floor.
# More information: https://clover.coex.tech/red_circle

# Input topic: main_camera/image_raw (camera image)
# Output topics:
#   cv/mask (red color mask)
#   cv/red_circle (position of the center of the red circle in 3D space)

import rospy
import cv2
import numpy as np
from math import nan
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
from clover import long_callback, srv
import tf2_ros
import tf2_geometry_msgs

rospy.init_node('cv', disable_signals=True) # disable signals to allow interrupting with ctrl+c

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)

bridge = CvBridge()

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

mask_pub = rospy.Publisher('~mask', Image, queue_size=1)
point_pub = rospy.Publisher('~red_circle', PointStamped, queue_size=1)

# read camera info
camera_info = rospy.wait_for_message('main_camera/camera_info', CameraInfo)
camera_matrix = np.float64(camera_info.K).reshape(3, 3)
distortion = np.float64(camera_info.D).flatten()


def img_xy_to_point(xy, dist):
    xy = cv2.undistortPoints(xy, camera_matrix, distortion, P=camera_matrix)[0][0]

    # Shift points to center
    xy -= camera_info.width // 2, camera_info.height // 2

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]

    return Point(x=xy[0] * dist / fx, y=xy[1] * dist / fy, z=dist)

def get_center_of_mass(mask):
    M = cv2.moments(mask)
    if M['m00'] == 0:
        return None
    return M['m10'] // M['m00'], M['m01'] // M['m00']

follow_red_circle = False

@long_callback
def image_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # we need to use two ranges for red color
    mask1 = cv2.inRange(img_hsv, (0, 150, 150), (15, 255, 255))
    mask2 = cv2.inRange(img_hsv, (160, 150, 150), (180, 255, 255))

    # combine two masks using bitwise OR
    mask = cv2.bitwise_or(mask1, mask2)

    # publish the mask
    if mask_pub.get_num_connections() > 0:
        mask_pub.publish(bridge.cv2_to_imgmsg(mask, 'mono8'))

    # calculate x and y of the circle
    xy = get_center_of_mass(mask)
    if xy is None:
        return

    # calculate and publish the position of the circle in 3D space
    altitude = get_telemetry('terrain').z
    xy3d = img_xy_to_point(xy, altitude)
    target = PointStamped(header=msg.header, point=xy3d)
    point_pub.publish(target)

    if follow_red_circle:
        # follow the target
        setpoint = tf_buffer.transform(target, 'map', timeout=rospy.Duration(0.2))
        set_position(x=setpoint.point.x, y=setpoint.point.y, z=nan, yaw=nan, frame_id=setpoint.header.frame_id)

# process each camera frame:
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)

rospy.loginfo('Hit enter to follow the red circle')
input()
follow_red_circle = True
rospy.spin()
