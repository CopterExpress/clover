cmake_minimum_required(VERSION 3.0)
project(clover)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  genmsg
  rospy
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  led_msgs
  geographic_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  image_transport
  cv_bridge
  dynamic_reconfigure
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# https://github.com/mavlink/mavros/blob/7f1a8/mavros/CMakeLists.txt#L42
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

# Workaround for OpenCV 3/4 support
set(_opencv_version 4)
find_package(OpenCV ${_opencv_version} QUIET COMPONENTS calib3d imgproc)
if (NOT OpenCV_FOUND)
  message(STATUS "Did not find OpenCV 4, searching for OpenCV 3")
  set(_opencv_version 3)
endif()

find_package(OpenCV ${_opencv_version} REQUIRED
  COMPONENTS
    calib3d
    imgproc
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  State.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetTelemetry.srv
  Navigate.srv
  NavigateGlobal.srv
  SetAltitude.srv
  SetYaw.srv
  SetYawRate.srv
  SetPosition.srv
  SetVelocity.srv
  SetAttitude.srv
  SetRates.srv
  SetLEDEffect.srv
  Execute.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Flow.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/optical_flow.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(simple_offboard src/simple_offboard.cpp)

# PX4 already has rc and led targets, so we prefix ours with clover_
add_executable(clover_rc src/rc.cpp)

add_executable(camera_markers src/camera_markers.cpp)

add_executable(vpe_publisher src/vpe_publisher.cpp)

add_executable(clover_led src/led.cpp)

add_executable(shell src/shell.cpp)

target_link_libraries(simple_offboard
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

# Don't change actual binary names
set_target_properties(clover_rc PROPERTIES OUTPUT_NAME rc)

set_target_properties(clover_led PROPERTIES OUTPUT_NAME led)

target_link_libraries(clover_rc ${catkin_LIBRARIES})

target_link_libraries(camera_markers ${catkin_LIBRARIES})

target_link_libraries(vpe_publisher ${catkin_LIBRARIES})

target_link_libraries(clover_led ${catkin_LIBRARIES})

target_link_libraries(shell ${catkin_LIBRARIES})

add_dependencies(simple_offboard ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(clover_led ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(shell ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
install(TARGETS simple_offboard clover_rc camera_markers vpe_publisher clover_led shell clover
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

catkin_install_python(PROGRAMS src/selfcheck.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY examples DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY www DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# TODO: install www

# Only install udev rules when building a Debian package
# FIXME: Other operating systems may have other prefixes
string(FIND ${CMAKE_INSTALL_PREFIX} "/opt/ros" _PREFIX_INDEX)
if (${_PREFIX_INDEX} EQUAL 0)
  message(STATUS "Building as a Debian package - adding udev rules as installable files")
  install(FILES
    udev/99-px4fmu.rules
    DESTINATION /lib/udev/rules.d
  )
else()
  message(STATUS "Building in a user workspace - not adding udev rules")
endif()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/basic.test)
  add_rostest(test/offboard.test)
endif()
