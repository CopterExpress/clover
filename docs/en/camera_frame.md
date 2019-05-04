# Adjusting the position of the main camera

> **Note** Documentation for the versions [of image](microsd_images.md), starting with **0.15**. For earlier versions, see [documentation for version **0.14**](https://github.com/CopterExpress/clever/blob/v0.14/docs/ru/camera_frame.md).

Position and orientation of the main camera is determined in file `~/catkin_ws/src/clever/clever/launch/main_camera.launch`:

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0 0 -0.07 -1.5707963 0 3.1415926 base_link main_camera_optical"/>
```

This line sets static transformation between frame `base_link` ([corresponds to the flight controller housing](frames.md)) and the camera (`main_camera_optical`) in format:

```txt
shift_x shift_y shift_z yaw_angle pitch_angle roll_angle
```

The frame of the camera is set so that:

* **<font color=red>x</font>** points to the right in the picture;
* **<font color=green>y</font>** points down in the picture;
* **<font color=blue>z</font>** points away from the camera matrix plane.

Shifts are set in meters, angles — in radians. Correctness of the transformation set may be checked using [rviz](rviz.md).

## Settings for Clever

The first image — how a copter model looks in rviz with these settings, the second image — how Clever looks with the same settings.

### 1. The camera is facing downward, the flat cable — backward

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 -0.07 -1.5707963 0 3.1415926 base_link main_camera_optical"/>
```

<img src="../assets/camera_option_1_rviz.png" width=400>
<img src="../assets/camera_option_1_clever.jpg" width=400>

### 2. The camera is facing downwards, the flat cable — forward

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 -0.07 1.5707963 0 3.1415926 base_link main_camera_optical"/>
```

<img src="../assets/camera_option_2_rviz.png" width=400>
<img src="../assets/camera_option_2_clever.jpg" width=400>

### 3. The camera is facing upward, the flat cable — backward

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 0.07 1.5707963 0 0 base_link main_camera_optical"/>
```

<img src="../assets/camera_option_3_rviz.png" width=400>
<img src="../assets/camera_option_3_clever.jpg" width=400>

### 4. The camera is facing upward, the flat cable — forward

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 0.07 -1.5707963 0 0 base_link main_camera_optical"/>
```

<img src="../assets/camera_option_4_rviz.png" width=400>
<img src="../assets/camera_option_4_clever.jpg" width=400>