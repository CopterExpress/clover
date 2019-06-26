# Positioning with ArUco markers

`aruco_pose` package consists of two nodelets: `aruco_detect` detects individual ArUco-markers and estimates their poses, `aruco_map` detects maps of markers using `aruco_detect` output.

## Quick start

To run a camera nodelet, markers and maps detector:

```bash
roslaunch aruco_pose sample.launch
```

You're going to need [`cv_camera`](http://wiki.ros.org/cv_camera) package installed.

## aruco_detect nodelet

`aruco_detect` detects ArUco markers on the image, publishes list of them (with poses), TF transformations, visualization markers and processed image for debugging.

It's recommended to run it within the same nodelet manager with the camera nodelet (e. g. [`cv_camera`](http://wiki.ros.org/cv_camera)).

### Parameters

* `~dictionary` (*int*) – ArUco dictionary (default: 2)
  * 0 = DICT_4X4_50
  * 1 = DICT_4X4_100,
  * 2 = DICT_4X4_250,
  * 3 = DICT_4X4_1000,
  * 4 = DICT_5X5_50,
  * 5 = DICT_5X5_100,
  * 6 = DICT_5X5_250,
  * 7 = DICT_5X5_1000,
  * 8 = DICT_6X6_50,
  * 9 = DICT_6X6_100,
  * 10 = DICT_6X6_250,
  * 11 = DICT_6X6_1000,
  * 12 = DICT_7X7_50,
  * 13 = DICT_7X7_100,
  * 14 = DICT_7X7_250,
  * 15 = DICT_7X7_1000,
  * 16 = DICT_ARUCO_ORIGINAL
* `~estimate_poses` (*bool*) – estimate single markers' poses (default: true)
* `~send_tf` (*bool*) – send TF transforms (default: true)
* `~frame_id_prefix` (*string*) – prefix for TF transforms names, marker's ID is appended (default: `aruco_`)
* `~length` (*double*) – markers' sides length
* `~length_override` (*map*) – lengths of markers with specified ids
* `~known_tilt` (*string*) – known tilt (pitch and roll) of all the markers as a frame

### Topics

#### Subscribed

* `image_raw` (*sensor_msgs/Image*) – camera image
* `camera_info` (*sensor_msgs/CameraInfo*) – camera calibration info

#### Published

* `~markers` (*aruco_pose/MarkerArray*) – list of detected markers with their corners and poses
* `~visualization` (*visualization_msgs/MarkerArray*) – visualization markers for rviz
* `~debug` (*sensor_msgs/Image*) – debug image with detected markers

### Published transforms

* `<camera_frame>` => `<frame_id_prefix><id>` – markers' poses

## aruco_map nodelet

`aruco_map` nodelet estimates position of markers map.

### Parameters

* `~map` – path to text file with markers list
* `~frame_id` – published frame id (default: `aruco_map`)
* `~known_tilt` – debug image width
* `~image_width` – debug image width (default: 2000)
* `~image_height` – debug image height (default: 2000)
* `~image_margin` – debug image margin (default: 200)
* `~dictionary` (*int*) – ArUco dictionary (default: 2) - should be the same as `dictionary` parameter of `aruco_detect` nodelet

Map file has one marker per line with the following line format:

```
marker_id marker_length x y z yaw pitch roll
```

Where yaw, pitch and roll are extrinsic rotation around Z, Y, X axis, respectively.

See examples in [`map`](map/) directory.

### Topics

#### Subscribed

* `image_raw` (*sensor_msgs/Image*) – camera image (used for debug image)
* `camera_info` (*sensor_msgs/CameraInfo*) – camera calibration info (used for debug image)
* `markers` (*aruco_pose/MarkerArray*) – list of markers detected by `aruco_pose` nodelet

#### Published

* `~pose` (*geometry_msgs/PoseWithCovarianceStamped*) – estimated map pose
* `~image` (*sensor_msgs/Image*) – planarized map image
* `~visualization` (*visualization_msgs/MarkerArray*) – markers map visualization for rviz
* `~debug` (*sensor_msgs/Image*) – debug image with detected markers and map axis

### Published transforms

* `<camera_frame>` => `<map_name>` – markers map pose

## Running tests

Command for running tests:

```bash
catkin_make run_tests && catkin_test_results
```

## Copyright

Copyright © 2018 Copter Express Technologies. Author: Oleg Kalachev.

Distributed under MIT License (https://opensource.org/licenses/MIT).
