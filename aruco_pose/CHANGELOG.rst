^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aruco_pose
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.21.1 (2020-11-17)
-------------------
* Merge pull request `#244 <https://github.com/deadln/clover/issues/244>`_ from goldarte/genmap-update
  genmap.py: Add x0 and y0 shift for markers coordinates
* genmap.py: Add x0 and y0 shift for markers coordinates
* aruco_pose nodelet cleanup (`#239 <https://github.com/deadln/clover/issues/239>`_)
  * aruco_pose: Unhardcode contour refinement
  Besides, this was basically a no-op anyway, since dynamic parameters
  overwrote that anyway.
  * aruco_pose: Late-construct objects that use ROS
  * aruco_map: Don't create/store node handle
  * aruco_pose: Don't assume dist_coeffs size
  * aruco_pose: more const == more better
  * aruco_pose: Be more obvious about changing variables
  * aruco_pose: Fix building for Kinetic
  * aruco_pose: Remove global add_definitions
* aruco_pose: Prevent OpenCV from crashing (`#238 <https://github.com/deadln/clover/issues/238>`_)
  * aruco_pose: Add tests that crash OpenCV
  * aruco_pose: Don't try to interpolate single points
* Continue renaming to Clover
* genmap.py: print column names in output
* genmap.py: make <first> argument not required
* genmap.py: add example usage
* genmap.py: make top-left by default (`#220 <https://github.com/deadln/clover/issues/220>`_)
  * genmap.py: make top-left by default
  * docs: make top left by default in genmap
  * genmap.py: fix usage string
* aruco_pose: dynamic reconfiguration of aruco detector (`#180 <https://github.com/deadln/clover/issues/180>`_)
  * aruco_detect: dynamic reconfiguration
  * aruco_pose: Depend on dynamic_reconfigure
  * aruco_pose: Use c++11 features instead of Boost
  * aruco_pose: Rearrange parameters, reset to OpenCV defaults
  * Update constrains for some parameters
  * aruco_pose: don’t hard-cord defaults for dynamic reconfigure
  * aruco_pose: add missing parameters
  * aruco_pose: fix tests
  * aruco_pose: typo
  * aruco_pose: fix
  * aruco_pose: fix test
  * aruco_pose: hardcode some new dynamic reconfigure parameters
  Co-authored-by: Alexey Rogachevskiy <sfalexrog@gmail.com>
  Co-authored-by: Arthur Golubtsov <goldartt@gmail.com>
* aruco_pose, clever: Minor cleanups
* Move to Raspbian Buster (`#193 <https://github.com/deadln/clover/issues/193>`_)
  * builder: Build against Buster
  * builder: Use correct repository specifications
  * builder: Move ld.so.preload to have less errors
  * builder: Use coex repo to install Monkey
  * builder: Search for buster ROS packages
  * aruco_pose: Vendor in aruco library from OpenCV 3.4.6
  * builder: Move to ROS Melodic
  * builder: Update kernel version
  * aruco_pose, clever: Remove opencv3 ROS dependency
  * builder: Update rosdep
  * travis: Disable eclint for vendored aruco library
  * tests: Don't try to locate opencv in ros
  * roscore: Use melodic distribution
  * Revert "aruco_pose: Vendor in aruco library from OpenCV 3.4.6"
  This reverts commit 9c14a8c002bb3396f9a7d9b2ba39969207f066ba.
  * aruco_pose: Vendor opencv_contrib/aruco again
  * builder: Add led packages
  * builder: Remove unused builder code
  * travis: Add native tests
  * builder: Set permissions for standalone-install
  * builder: Use -y for package installation
  * builder: Add repo for standalone build
  * builder: Use correct file types for standalone install
  * aruco_pose: Accept rgb8 map images
  * builder: Disable mjpg_streamer test
  * aruco_pose: Allow rgb8 map images (again)
  * builder: Re-add mjpgstreamer
  * builder: Install tornado==4.2.1 for rosbridge_suite
  * builder: Use more recent base image
  * builder: Use default kernel
  * builder: Move ld.so.preload back after tests
  * builder: Disable catkin tests
  These tests fail on a remote machine but seem to pass just fine on real hardware. Something must have changed between Kinetic and Melodic, and we must investigate more, but for now we just need a working image.
  * aruco_pose: Remove unused vendored code
  * selfcheck: Update systemd-analyze regex
  * builder: Add opencv repository
  * rosdep: Update package definitions for Melodic
  * rosdep: Use proper yaml formatting
  * travis: Remove unnecessary space
  * docs: Reference Melodic wherever possible
* aruco_pose: Allow rgb8 map images
* aruco_pose: improve processing of axis visualization in aruco_map/image (`#167 <https://github.com/deadln/clover/issues/167>`_)
  * aruco_pose: improved processing of axis visualization in the topic /aruco_map/image
  * aruco_pose: reworked method for drawing axis arrow
  * aruco_pose: fix indentation error
  * Fix style
* clever.launch: change log formatting (`#175 <https://github.com/deadln/clover/issues/175>`_)
  * clever.launch: Add node name to output format
  * selfcheck: Use new log format
  * aruco_pose, clever: Remove node names from messages
  * aruco_pose, clever: Use nodelet-aware rosconsole macros
  * clever.launch: Use logger name instead of node name
  * clever.launch: change rosconsole format a little
* aruco_pose: don’t publish transforms of markers in the map
* aruco_map: add markers topic
* Add editorconfig-checker (`#149 <https://github.com/deadln/clover/issues/149>`_)
  * Add editorconfig-checker
  * Editorconfig-check fix
  * Remove temporal cat
* aruco_map: add image_axis parameter for drawing axis on ~image topic
* Use aruco_map\_ prefix for markers in the map
* Forgotten lines
* aruco_map: fix includes order
* aruco_map: possibility to publish static transforms for map's markers
* aruco_pose: add testing markers' tf frames
* aruco_pose: require all the nodelets not to crash in tests
* Use pytest for tests (`#133 <https://github.com/deadln/clover/issues/133>`_)
  * aruco_pose: use pytest
  * Use ros_pytest
  * Add ros_pytest to rosdep
  * aruco_pose: compare floats more roughly in pytest
  * aruco_pose: rewrite all the rest tests in pytest
* aruco_pose: remove unused lines from tests
* aruco_pose: fix crashing the nodelet if markers on the map are to small
* aruco_pose: fix command for running tests in readme
* Update sources hats
* Remove bad 17 marker from cmit aruco map
* aruco_pose: Add cmit aruco map
* Revert "aruco_map: Use two-pass solvePnP"
  This reverts commit 91f6f6dd3292ebde5dda375512a9e05c111de124. Additional testing revealed this "fix" to provide incorrect results.
* Merge pull request `#125 <https://github.com/deadln/clover/issues/125>`_ from sfalexrog/WIP/aruco_fix
  aruco_map: Use two-pass solvePnP
* aruco_map: Use two-pass solvePnP
  There are cases when iterative solvePnP method converges on a "wrong" camera position due to projectPoints not treating negative Z values properly.
  Other methods don't seem to be affected by that, but their results differ from the iterative method slightly. By combining these two methods we
  "nudge" the iterative method towards the correct camera position and get satisfactory results most of the time.
  Sometimes, though, even with the "nudge" the iterative method diverges catastrophically, and this is not caught by the solver. We work around that
  by assuming our camera position cannot be too far from the markers.
* aruco_map: map parser improvements (`#118 <https://github.com/deadln/clover/issues/118>`_)
  * aruco_map: Improve parser
  * aruco_map: Use marker id for map visualization
  * aruco_pose: Add parser pass test
  * aruco_map: Code style
  * aruco_pose: Add more test cases
  * aruco_map: Better message handling
  * aruco_map: Be more informative about bad lines
  * aruco_map: Add failure mode tests
  * aruco_map: Be less strict about map contents
  * aruco_pose: Restructure tests
  * aruco_map: Don't use marker id in visualization
  * aruco_map: Check for marker uniqueness
  * aruco_pose: Use board data to reject duplicate markers
  * aruco_pose/test: Spelling fixes
* dictionary parameter of aruco_map nodelet
  aruco_map nodelet has it's own dictionary parameter
* aruco_pose: Try to draw as much of axes as possible
* aruco_pose/draw: Be more strict about drawing axis
* aruco_pose: little style fix
* aruco_pose: add param auto_flip
* aruco_pose: fix snapping alrogithm
* aruco_pose: fix office ceiling map
* aruco_map: Try to fix frame drawing bug
  `cv::aruco::drawAxis` would attempt to draw detected frame origin even when it's behind the camera. This resulted in an invalid, flipped frame displayed in `/aruco_map/debug`.
  This commit prevents drawing frame axis if the frame origin (projected to the screen space) is behind the screen plane.
* genmap.py: remove unused arguments
* Fix top left setting
* Remove marker 17 as of too common faults + remove CRLF
* Typo
* Add genmap.py tool
* aruco_map: fix drawing maps with pitch/roll rotated markers
* aruco_pose: remove irrelevant comment
* aruco_pose: loose assertion even more
* aruco_pose: loose floats assertion for make tests pass
* Add office ceiling aruco map
* aruco_pose: don’t use pytest
* aruco_pose: add test dependency image_publisher
* builder: run catkin_ws packages tests
* aruco_pose: moar tests
* aruco_pose: improve tests
* aruco_pose: add length to Marker message
* spaces -> tabs
* aruco_map: fix drawing map image
* aruco_pose: fixes to readme
* Typo
* aruco_pose: add readme
* aruco_map: remove unused parameter
* aruco_pose: add sample launch file
* aruco_pose: use Pose instead of PoseWithCovariance in Marker message (for now)
* aruco_pose: fix tests
* aruco_pose: draw detected markers in aruco_map/debug topic
* aruco_map: align marker map by z axis
* Add aruco_map/debug topic
* aruco_pose: style fix
* aruco_pose: remove using namespace from .h-file
* aruco_pose: add sample office map
* aruco_pose: fix generating gridboard
* aruco_pose: rename known_orientation to known_tilt
* aruco_pose: more tests
* aruco_pose: rename snap_orientation to known_orientation
* aruco_pose: add basic tests
* aruco_detect: add length_override parameter for overriding individual marker’s length
* aruco_map: parametrize output image width, height and margin
* aruco_map: enable rotating (yaw, pitch, roll) each marker in the map
* Fix
* aruco_map: publish visualization markers
* aruco_detect: fix single markers frame (with snapping)
* aruco_pose: cleanup package.xml and change format to 2
* Refactor aruco_pose, split up to aruco_detect and aruco_map notelets
* Merge branch 'master' into master
* Contributors: Alamoris, Alexey Rogachevskiy, Arthur Golubtsov, Ilya Petrov, Oleg Kalachev, sfalexrog

0.15.1 (2019-01-24)
-------------------
* aruco_pose: Use find_package instead of hardcoded opencv paths
* Merge branch 'master' into CL3_assemble_new
* Merge pull request `#68 <https://github.com/deadln/clover/issues/68>`_ from urpylka/master
  New structure of the CLEVER
* image_builder: TEMP: Add support installation wo sources
* Contributors: Artem Smirnov, Arthur Golubtsov, sfalexrog

0.11.4 (2018-09-19 21:32)
-------------------------

0.11.3 (2018-09-19 08:16)
-------------------------

0.11.2 (2018-09-19 07:04)
-------------------------

0.11.1 (2018-09-19 04:40)
-------------------------

0.10.1 (2018-09-05 21:15)
-------------------------

0.8.1 (2018-07-25)
------------------
* aruco_pose: publish debug image even where there is no board
* aruco_pose: undocumented possibility to set custom markers board
  parameters:
  ~type=custom
  ~markers
* Track 411 files into repository.
  - untracked .gitattributes
  - untracked .gitignore
  - untracked apps/ios/.gitignore
  - untracked apps/ios/cleverrc.xcodeproj/project.pbxproj
  - untracked apps/ios/cleverrc.xcodeproj/project.xcworkspace/contents.xcworkspacedata
  - untracked apps/ios/cleverrc.xcworkspace/contents.xcworkspacedata
  - untracked apps/ios/cleverrc/AppDelegate.swift
  - untracked apps/ios/cleverrc/Assets.xcassets/AppIcon.appiconset/cleverios180-1.png
  - untracked apps/ios/cleverrc/Assets.xcassets/AppIcon.appiconset/cleverios180.png
  - untracked apps/ios/cleverrc/Assets.xcassets/AppIcon.appiconset/Contents.json
  - untracked apps/ios/cleverrc/Assets.xcassets/Contents.json
  - untracked apps/ios/cleverrc/Assets.xcassets/Image.imageset/Contents.json
  - untracked apps/ios/cleverrc/Base.lproj/LaunchScreen.storyboard
  - untracked apps/ios/cleverrc/Base.lproj/Main.storyboard
  - untracked apps/ios/cleverrc/BinUtils.swift
  - untracked apps/ios/cleverrc/clever.svg
  - untracked apps/ios/cleverrc/index.html
  - untracked apps/ios/cleverrc/Info.plist
  - untracked apps/ios/cleverrc/main.css
  - untracked apps/ios/cleverrc/main.js
  - untracked apps/ios/cleverrc/README.md
  - untracked apps/ios/cleverrc/roslib.js
  - untracked apps/ios/cleverrc/telemetry.js
  - untracked apps/ios/cleverrc/ViewController.swift
  - untracked apps/ios/Podfile
  - untracked apps/ios/Podfile.lock
  - untracked apps/ios/README.md
  - untracked aruco_pose/CMakeLists.txt
  - untracked aruco_pose/nodelet_plugins.xml
  - untracked aruco_pose/package.xml
  - untracked aruco_pose/src/aruco_pose.cpp
  - untracked aruco_pose/src/fix.cpp
  - untracked assets/11_1.png
  - untracked assets/11_2.png
  - untracked assets/11_3.png
  - untracked assets/11_4.png
  - untracked assets/11_5.png
  - untracked assets/13_1.png
  - untracked assets/13_10.png
  - untracked assets/13_11.png
  - untracked assets/13_2.png
  - untracked assets/13_3.jpg
  - untracked assets/13_4.png
  - untracked assets/13_5.png
  - untracked assets/13_6.png
  - untracked assets/13_7.png
  - untracked assets/13_8.png
  - untracked assets/13_9.png
  - untracked assets/15_1.png
  - untracked assets/15_2.png
  - untracked assets/15_3.png
  - untracked assets/15_4.png
  - untracked assets/15_5.png
  - untracked assets/15_6.png
  - untracked assets/15_7.png
  - untracked assets/16_1.png
  - untracked assets/16_2.png
  - untracked assets/16_3.png
  - untracked assets/16_4.png
  - untracked assets/1_1.png
  - untracked assets/1_10.png
  - untracked assets/1_11.png
  - untracked assets/1_12.png
  - untracked assets/1_13.png
  - untracked assets/1_2.png
  - untracked assets/1_3.png
  - untracked assets/1_4.png
  - untracked assets/1_5.png
  - untracked assets/1_6.png
  - untracked assets/1_7.png
  - untracked assets/1_8.png
  - untracked assets/1_9.png
  - untracked assets/2_1.png
  - untracked assets/2_2.png
  - untracked assets/2_3.png
  - untracked assets/2_4.png
  - untracked assets/2_5.png
  - untracked assets/2_6.png
  - untracked assets/2_7.png
  - untracked assets/2_8.png
  - untracked assets/2_9.png
  - untracked assets/4_1.png
  - untracked assets/4_2.png
  - untracked assets/4_3.png
  - untracked assets/4_4.png
  - untracked assets/4_5.png
  - untracked assets/4_6.png
  - untracked assets/7_1.png
  - untracked assets/7_2.png
  - untracked assets/7_3.png
  - untracked assets/7_4.png
  - untracked assets/8_1.png
  - untracked assets/8_2.png
  - untracked assets/8_3.png
  - untracked assets/8_4.png
  - untracked assets/8_5.png
  - untracked assets/8_6.png
  - untracked assets/9_1.png
  - untracked assets/9_2.png
  - untracked assets/addEqipment.jpg
  - untracked assets/airframeSetup.jpg
  - untracked assets/allElements.png
  - untracked assets/attentionSave.jpg
  - untracked assets/brrc2205.png
  - untracked assets/brrc2205on.png
  - untracked assets/brrc2205ondeck.png
  - untracked assets/calibrateaxcel.jpg
  - untracked assets/calibrateaxcelstart.jpg
  - untracked assets/calibratecompass.jpg
  - untracked assets/calibrateESC.jpg
  - untracked assets/calibrategyro.jpg
  - untracked assets/calibratePIDparams.jpg
  - untracked assets/calibrateView.jpg
  - untracked assets/calibrateViewStart.jpg
  - untracked assets/casebattery.png
  - untracked assets/chooseSwitch.jpg
  - untracked assets/Clever main.png
  - untracked assets/clever.jpg
  - untracked assets/Clevermain.png
  - untracked assets/connectBattery.png
  - untracked assets/connectingRadio.png
  - untracked assets/connectionESCtoReceiver.png
  - untracked assets/connectionLost.jpg
  - untracked assets/connectionOK.jpg
  - untracked assets/connectionPixhawk.png
  - untracked assets/consistofTransmitter.jpg
  - untracked assets/cutwire14AWG.jpg
  - untracked assets/escDYSzap.png
  - untracked assets/escWires.png
  - untracked assets/explosion.png
  - untracked assets/firmwarePX4.jpg
  - untracked assets/flightModes.jpg
  - untracked assets/helphand.jpg
  - untracked assets/holderLegs.png
  - untracked assets/isoViewmountHolder.png
  - untracked assets/jumper.png
  - untracked assets/keep.png
  - untracked assets/lockradio.jpg
  - untracked assets/lockradio.png
  - untracked assets/lowsafeDeck.png
  - untracked assets/mainWindow.jpg
  - untracked assets/motorsTopview.png
  - untracked assets/mount5vconnector.png
  - untracked assets/mountAntenna.png
  - untracked assets/mountBeams.png
  - untracked assets/mountBottomDeck.png
  - untracked assets/mountHolder.png
  - untracked assets/mountPDB.png
  - untracked assets/mountReceiverDeck.png
  - untracked assets/mountReceiverStud.png
  - untracked assets/mountxt60pinsocket.png
  - untracked assets/notmoveslider.jpg
  - untracked assets/pixhawk.png
  - untracked assets/radioTransmitter.png
  - untracked assets/readyBatteryholder.png
  - untracked assets/receiver5V.png
  - untracked assets/receiverPPM.png
  - untracked assets/resolderingESC.png
  - untracked assets/safehighRadial.png
  - untracked assets/safeLegs.png
  - untracked assets/safelowRadial.png
  - untracked assets/safetybyassem.png
  - untracked assets/safetyINflight.png
  - untracked assets/safetyPower.png
  - untracked assets/safetyPreflight.png
  - untracked assets/soldering5VTOpdb.png
  - untracked assets/solderingBrrc2205ondeckTOescDYSzap.png
  - untracked assets/solderingPowerwires.png
  - untracked assets/solderingxt60socketTOpdb.png
  - untracked assets/stand.jpg
  - untracked assets/startPDBtest.jpg
  - untracked assets/testMotors.png
  - untracked assets/topESCcaseview.png
  - untracked assets/topPreview.png
  - untracked assets/topviewmountPDB.png
  - untracked assets/topviewpixhawk.png
  - untracked assets/turnoffSafetyswitch.jpg
  - untracked assets/xt60pinsocket.jpg
  - untracked assets/zap.jpg
  - untracked assets/zapPDBtest.jpg
  - untracked clever/camera_info/fisheye_cam_320.yaml
  - untracked clever/camera_info/fisheye_cam_640.yaml
  - untracked clever/CMakeLists.txt
  - untracked clever/launch/arduino.launch
  - untracked clever/launch/aruco.launch
  - untracked clever/launch/clever.launch
  - untracked clever/launch/copter_visualization.launch
  - untracked clever/launch/fpv_camera.launch
  - untracked clever/launch/main_camera.launch
  - untracked clever/launch/mavros.launch
  - untracked clever/launch/sitl.launch
  - untracked clever/launch/web_server.launch
  - untracked clever/nodelet_plugins.xml
  - untracked clever/package.xml
  - untracked clever/requirements.txt
  - untracked clever/src/aruco_vpe.cpp
  - untracked clever/src/fcu_horiz.cpp
  - untracked clever/src/fpv_camera
  - untracked clever/src/global_local.py
  - untracked clever/src/rc.cpp
  - untracked clever/src/simple_offboard.py
  - untracked clever/src/util.h
  - untracked clever/src/util.py
  - untracked clever/src/web_server.py
  - untracked clever/srv/GetTelemetry.srv
  - untracked clever/srv/Navigate.srv
  - untracked clever/srv/SetAttitude.srv
  - untracked clever/srv/SetAttitudeYawRate.srv
  - untracked clever/srv/SetPosition.srv
  - untracked clever/srv/SetPositionGlobal.srv
  - untracked clever/srv/SetPositionGlobalYawRate.srv
  - untracked clever/srv/SetPositionYawRate.srv
  - untracked clever/srv/SetRates.srv
  - untracked clever/srv/SetRatesYaw.srv
  - untracked clever/srv/SetVelocity.srv
  - untracked clever/srv/SetVelocityYawRate.srv
  - untracked deploy/clever.service
  - untracked deploy/clever_arudino.tar.gz
  - untracked deploy/generate_ros_lib
  - untracked deploy/roscore.env
  - untracked deploy/roscore.service
  - modified docs/3g.md
  - modified docs/assemble.md
  - untracked docs/deck.md
  - modified docs/etcher.md
  - modified docs/frames.md
  - modified docs/les1.md
  - modified docs/les11.md
  - modified docs/les13.md
  - modified docs/les15.md
  - modified docs/les16.md
  - modified docs/les2.md
  - modified docs/les4.md
  - modified docs/les7.md
  - modified docs/les8.md
  - modified docs/les9.md
  - modified docs/modes.md
  - untracked docs/powerConnection.md
  - modified docs/radioerrors.md
  - modified docs/radioerrors1.md
  - modified docs/safety.md
  - modified docs/setup.md
  - modified docs/simple_offboard.md
  - modified docs/tb.md
  - untracked docs/testConnection.md
  - modified docs/wifi.md
  - untracked docs/zap.md
  - removed gpsmd.md
  - untracked image/apps.sh
  - untracked image/git_release.py
  - untracked image/iface.sh
  - untracked image/image-config.sh
  - untracked image/Jenkinsfile
  - untracked image/ros.sh
  - untracked image/yadisk.py
  - removed img/11_1.png
  - removed img/11_2.png
  - removed img/11_3.png
  - removed img/11_4.png
  - removed img/11_5.png
  - removed img/13_1.png
  - removed img/13_10.png
  - removed img/13_11.png
  - removed img/13_2.png
  - removed img/13_3.jpg
  - removed img/13_4.png
  - removed img/13_5.png
  - removed img/13_6.png
  - removed img/13_7.png
  - removed img/13_8.png
  - removed img/13_9.png
  - removed img/15_1.png
  - removed img/15_2.png
  - removed img/15_3.png
  - removed img/15_4.png
  - removed img/15_5.png
  - removed img/15_6.png
  - removed img/15_7.png
  - removed img/16_1.png
  - removed img/16_2.png
  - removed img/16_3.png
  - removed img/16_4.png
  - removed img/1_1.png
  - removed img/1_10.png
  - removed img/1_11.png
  - removed img/1_12.png
  - removed img/1_13.png
  - removed img/1_2.png
  - removed img/1_3.png
  - removed img/1_4.png
  - removed img/1_5.png
  - removed img/1_6.png
  - removed img/1_7.png
  - removed img/1_8.png
  - removed img/1_9.png
  - removed img/2_1.png
  - removed img/2_2.png
  - removed img/2_3.png
  - removed img/2_4.png
  - removed img/2_5.png
  - removed img/2_6.png
  - removed img/2_7.png
  - removed img/2_8.png
  - removed img/2_9.png
  - removed img/4_1.png
  - removed img/4_2.png
  - removed img/4_3.png
  - removed img/4_4.png
  - removed img/4_5.png
  - removed img/4_6.png
  - removed img/7_1.png
  - removed img/7_2.png
  - removed img/7_3.png
  - removed img/7_4.png
  - removed img/8_1.png
  - removed img/8_2.png
  - removed img/8_3.png
  - removed img/8_4.png
  - removed img/8_5.png
  - removed img/8_6.png
  - removed img/9_1.png
  - removed img/9_2.png
  - removed img/addEqipment.jpg
  - removed img/airframeSetup.jpg
  - removed img/allElements.png
  - removed img/attentionSave.jpg
  - removed img/brrc2205.png
  - removed img/brrc2205on.png
  - removed img/brrc2205ondeck.png
  - removed img/calibrateaxcel.jpg
  - removed img/calibrateaxcelstart.jpg
  - removed img/calibratecompass.jpg
  - removed img/calibrateESC.jpg
  - removed img/calibrategyro.jpg
  - removed img/calibratePIDparams.jpg
  - removed img/calibrateView.jpg
  - removed img/calibrateViewStart.jpg
  - removed img/casebattery.png
  - removed img/chooseSwitch.jpg
  - removed img/Clever main.png
  - removed img/clever.jpg
  - removed img/Clevermain.png
  - removed img/connectBattery.png
  - removed img/connectingRadio.png
  - removed img/connectionESCtoReceiver.png
  - removed img/connectionLost.jpg
  - removed img/connectionOK.jpg
  - removed img/connectionPixhawk.png
  - removed img/consistofTransmitter.jpg
  - removed img/cutwire14AWG.jpg
  - removed img/escDYSzap.png
  - removed img/escWires.png
  - removed img/explosion.png
  - removed img/firmwarePX4.jpg
  - removed img/flightModes.jpg
  - removed img/helphand.jpg
  - removed img/holderLegs.png
  - removed img/isoViewmountHolder.png
  - removed img/jumper.png
  - removed img/keep.png
  - removed img/lockradio.jpg
  - removed img/lockradio.png
  - removed img/lowsafeDeck.png
  - removed img/mainWindow.jpg
  - removed img/motorsTopview.png
  - removed img/mount5vconnector.png
  - removed img/mountAntenna.png
  - removed img/mountBeams.png
  - removed img/mountBottomDeck.png
  - removed img/mountHolder.png
  - removed img/mountPDB.png
  - removed img/mountReceiverDeck.png
  - removed img/mountReceiverStud.png
  - removed img/mountxt60pinsocket.png
  - removed img/notmoveslider.jpg
  - removed img/pixhawk.png
  - removed img/radioTransmitter.png
  - removed img/readyBatteryholder.png
  - removed img/receiver5V.png
  - removed img/receiverPPM.png
  - removed img/resolderingESC.png
  - removed img/safehighRadial.png
  - removed img/safeLegs.png
  - removed img/safelowRadial.png
  - removed img/safetybyassem.png
  - removed img/safetyINflight.png
  - removed img/safetyPower.png
  - removed img/safetyPreflight.png
  - removed img/soldering5VTOpdb.png
  - removed img/solderingBrrc2205ondeckTOescDYSzap.png
  - removed img/solderingPowerwires.png
  - removed img/solderingxt60socketTOpdb.png
  - removed img/stand.jpg
  - removed img/startPDBtest.jpg
  - removed img/testMotors.png
  - removed img/topESCcaseview.png
  - removed img/topPreview.png
  - removed img/topviewmountPDB.png
  - removed img/topviewpixhawk.png
  - removed img/turnoffSafetyswitch.jpg
  - removed img/xt60pinsocket.jpg
  - removed img/zap.jpg
  - removed img/zapPDBtest.jpg
  - removed notes/deck.md
  - removed notes/powerConnection.md
  - removed notes/testConnection.md
  - removed notes/zap.md
  - removed primeri-programm.md
  - modified README.md
  - removed sborka.md
  - removed sitl.md
  Auto commit by GitBook Editor
* Merge branch 'mergebranch' of ../clever_bundle into mergebranch
* ~markers_sep_x, ~markers_sep_y parameters for grid boards + various fixes
* Show exception if creating aruco board fails
* Fix aruco board «reference point» determination
* Continue «fixing» weird bugs in aruco functions
* Always publish /aruco_pose/debug
* Cleanup code
* markers_ids settings for grid boards
* Merge branch 'aruco_pose'
* First version of aruco_pose nodelet
* Contributors: Oleg Kalachev, urpylka
