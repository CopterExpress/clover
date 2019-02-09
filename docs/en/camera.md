# Working with the camera

To work with the main camera, make sure it is enabled in file `~/catkin_ws/src/clever/clever/launch/clever.launch`:

```xml
<arg name="main_camera" default="true"/>
```

Also make sure that [correct position and orientation are indicated] for the camera (camera_frame.md).

The `clever` package must be restarted after the launch-file has been edited:

```(bash)
sudo systemctl restart clever
```

For monitoring images from the camera, you may use rqt or [web_video_server](web_video_server.md).

## Computer vision

For implementation of the computer vision algorithms, it is recommended to use the [OpenCV] library that is pre-installed in [the SD card image] (microsd_images.md) (https://opencv.org).

### Python

Main article: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython.

An example of creating a subscriber for a topic with an image from the main camera for processing with OpenCV:

```python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('computer_vision_sample')
bridge = CvBridge()

def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    # Do any image processing with cv2...

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

rospy.spin()
```

To debug image processing, you can publish a separate topic with the processed image:

```python
image_pub = rospy.Publisher('~debug', Image)
```

Publishing the processed image (at the end of the image_callback function):

```python
image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
```

The obtained images can be viewed using [web_video_server](web_video_server.md).

### Examples

#### Working with QR codes

> **Hint** For high-speed recognition and positioning, it is better to use [ArUco markers](aruco.md).

To program actions of the copter upon detection of [QR codes](https://en.wikipedia.org/wiki/QR_code) you can use the [ZBar] library (http://zbar.sourceforge.net). It should be installed using pip:

```(bash)
sudo pip install zbar
```

Recognizing QR codes in Python:

```python
import cv2
import zbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
scanner = zbar.ImageScanner()
scanner.parse_config('enable')

# Image subscriber callback function
def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY, dstCn=0)

    pil = ImageZ.fromarray(gray)
    raw = pil.tobytes()

    image = zbar.Image(320, 240, 'Y800', raw)  # Image params
    scanner.scan(image)

    for symbol in image:
        # print detected QR code
        print 'decoded', symbol.type, 'symbol', '"%s"' % symbol.data

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
```

The script will take up to 100% CPU capacity. To slow down the script artificially, you can run [throttling](http://wiki.ros.org/topic_tools/throttle) of frames from the camera, for example, at 5 Hz (`main_camera.launch`):

```xml
<node pkg="topic_tools" name="cam_throttle" type="throttle"
    args="messages main_camera/image_raw 5.0 main_camera/image_raw/throttled"/>
```

The topic for the subscriber in this case should be changed for `main_camera/image_raw/throttled`.
