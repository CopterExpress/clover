# ArUco markers

> **Note** The following applies to [image versions](image.md) **0.16** and up. Older documentation is still available for [for version **0.15.1**](https://github.com/CopterExpress/clever/blob/v0.15.1/docs/ru/aruco.md).

[ArUco markers](https://docs.opencv.org/3.2.0/d5/dae/tutorial_aruco_detection.html) are commonly used for vision-based position estimation.

Examples of ArUco markers:

![ArUco markers](../assets/markers.jpg)

> **Hint** Use the most matte paper for printing visual markers. Glossy paper may glitter in the light, severely deteriorating the quality of recognition.

For rapid generation of markers for printing, you may use an online tool: http://chev.me/arucogen/.

[Clever Raspberry Pi image](image.md) contains a pre-installed `aruco_pose` ROS package, which can be used for marker detection.

## Modes of operation

There are several preconfigured modes of operation for ArUco markers on the Clever drone:

* [single marker detection and navigation](aruco_marker.md);
* [map-based navigation](aruco_map.md).

> **Info** Additional documentation for the `aruco_pose` ROS package is available [on GitHub](https://github.com/CopterExpress/clever/blob/master/aruco_pose/README.md).
