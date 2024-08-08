^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clover
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.25 (2024-07-28)
-----------------
* Optimize displaying newlines in the topic viewer, add width and indent parameters.
* Link assets instead of copying in documentation to save space.
* Install image_geometry and dynamic_reconfigure as clover dependencies.
* Add dictionary parameter to aruco.launch.
* Solve the issue with aruco_detect not running when aruco_map is not enabled.
* Documentation improvements.
* Rest changes.

0.24 (2023-10-11)
-----------------
* Significant update to autonomous flights API.
* Updates to selfcheck.py.
* Support PX4 v1.14 parameters.
* Added scripts for automatic testing of autonomous flights.
* Added new examples for working with the camera, including a red circle model and its recognition and following.
* Implemented long_callback Python decorator to address the issue #218.
* Implemented optical_flow/enabled dynamic parameter.
* Updated LED strip native library to support RPi 4 rev. 1.5.
* Show number of messages received in web topic viewer.
* Run main_camera/image_raw_throttled topic by default.
* Added rectify argument to main_camera.launch
* Added udev rules for all supported autopilots by PX4.
* Various changes.

0.23 (2022-02-10)
-----------------
* Web tool for topics monitoring.
* Publish optical flow when local position is not available.
* Force estimator init.
* Web viewer for Clover logs.
* selfcheck.py improvements.
* Various changes.

0.22 (2021-06-07)
-----------------
* Move to ROS Noetic and Python 3.
* aruco.launch: add placement, length and map arguments.
* Web: add link for viewing the error log.
* LED: add error/ignore parameter to not flash on some errors.
* Wait for FC and camera devices before launching mavros and camera driver.
* clover.launch: disable rc node by default.
* optical_flow: publish debug image even when calc_flow_gyro failed.
* Various changes.

0.21.1 (2020-11-17)
-------------------
* First release of clover package to ROS
* Contributors: Alexey Rogachevskiy, Arthur Golubtsov, Oleg Kalachev
