cmake_minimum_required(VERSION 3.5)
project(clover_gazebo_plugins VERSION 1.0.0)

find_package(catkin COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  sensor_msgs
)

if (NOT ${gazebo_ros_FOUND})
  message(STATUS "gazebo_ros not installed, not building gazebo plugins")
  return()
endif()


#######################
## Find Dependencies ##
#######################

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

option(BUILD_ROS_INTERFACE "Enable building ROS dependent plugins" ON)

option(SEND_VISION_ESTIMATION_DATA "Send Mavlink VISION_POSITION_ESTIMATE msgs" OFF)
option(SEND_ODOMETRY_DATA "Send Mavlink ODOMETRY msgs" OFF)

# Set c++11 or higher
include(EnableC++XX)

find_package(gazebo)

## System dependencies are found with CMake's conventions
find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)
find_package(gazebo REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PythonInterp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TinyXML REQUIRED)

pkg_check_modules(OGRE OGRE)

if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
  include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
else()
  include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})

# find MAVLink
find_package(MAVLink)

# see if catkin was invoked to build this
if (CATKIN_DEVEL_PREFIX)
  message(STATUS "catkin ENABLED")
  find_package(catkin REQUIRED)
  if (catkin_FOUND)
    catkin_package()
  else()
    message(FATAL_ERROR "catkin not found")
  endif()
else()
  message(STATUS "catkin DISABLED")
endif()

# XXX this approach is extremely error prone
# it would be preferable to either depend on the
# compiled headers from Gazebo directly
# or to have something entirely independent.
#
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message(STATUS "Gazebo version: ${GAZEBO_VERSION}")
endif()

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(Eigen QUIET)
  if(NOT EIGEN_FOUND)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
  else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  endif()
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

###########
## Build ##
###########

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wno-deprecated-declarations")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

#include_directories(
#  include
#  ${Boost_INCLUDE_DIR}
#  ${CMAKE_CURRENT_BINARY_DIR}
#  ${EIGEN3_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
#  ${GAZEBO_INCLUDE_DIRS}
#  ${GAZEBO_MSG_INCLUDE_DIRS}
#  ${MAVLINK_INCLUDE_DIRS}
#  ${OGRE_INCLUDE_DIRS}
#  ${OGRE_INCLUDE_DIRS}/Paging		# Workaround for "fatal error: OgrePagedWorldSection.h: No such file or directory"
#  ${OpenCV_INCLUDE_DIRS}
#  ${OpticalFlow_INCLUDE_DIRS}
#  ${TinyXML_INCLUDE_DIRS}
#  )
#endif()

#link_libraries(
#  ${Boost_SYSTEM_LIBRARY_RELEASE}
#  ${Boost_THREAD_LIBRARY_RELEASE}
#  ${Boost_TIMER_LIBRARY_RELEASE}
#  ${GAZEBO_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  ${PROTOBUF_LIBRARY}
#  )

#link_directories(
#  ${GAZEBO_LIBRARY_DIRS}
#  ${CMAKE_CURRENT_BINARY_DIR}
#  ${OGRE_LIBRARY_DIRS}
#  )

#-----------#
# Functions #
#-----------#

set(plugins_msgs)

function(generate_protobuf_msgs msg_name)
  message(STATUS "Generating ${msg_name} from ${ARGN}")
  add_library(${msg_name} SHARED ${ARGN})
  target_link_libraries(${msg_name} PUBLIC ${PROTOBUF_LIBRARY})
  target_include_directories(${msg_name}
    PUBLIC
      ${Boost_INCLUDE_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}
      ${EIGEN3_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
      ${GAZEBO_INCLUDE_DIRS}
      ${GAZEBO_MSG_INCLUDE_DIRS}
      ${MAVLINK_INCLUDE_DIRS}
      ${OGRE_INCLUDE_DIRS}
      ${OGRE_INCLUDE_DIRS}/Paging		# Workaround for "fatal error: OgrePagedWorldSection.h: No such file or directory"
      ${OpenCV_INCLUDE_DIRS}
      ${TinyXML_INCLUDE_DIRS}
  )
  set(plugins_msgs ${plugins_msgs} ${msg_name} PARENT_SCOPE)
endfunction()

function(add_plugin plugin_name)
  add_library(${plugin_name} SHARED ${ARGN})
  target_link_libraries(${plugin_name}
    PUBLIC 
      ${plugins_msgs}
      ${Boost_LIBRARIES}
      ${GAZEBO_LIBRARIES}
      ${TinyXML_LIBRARIES}
      )
  target_include_directories(${plugin_name}
    PUBLIC
      include
      ${GAZEBO_LIBRARY_DIRS}
      ${CMAKE_CURRENT_BINARY_DIR}
      ${OGRE_LIBRARY_DIRS}) 
  #add_dependencies(${plugin_name} mav_msgs nav_msgs std_msgs sensor_msgs physics_msgs)
endfunction()

#--------------------#
# Message Generation #
#--------------------#

set(mav_msgs
  msgs/CommandMotorSpeed.proto
  msgs/MotorSpeed.proto
  )
set(nav_msgs msgs/Odometry.proto)
set(physics_msgs msgs/Wind.proto)
set(std_msgs msgs/Int32.proto)
set(sensor_msgs
  msgs/Imu.proto
  msgs/IRLock.proto
  msgs/Float.proto
  msgs/Groundtruth.proto
  msgs/Range.proto
  msgs/SITLGps.proto
  msgs/OpticalFlow.proto
  msgs/MagneticField.proto
  msgs/Pressure.proto
  )

PROTOBUF_GENERATE_CPP(MAV_PROTO_SRCS MAV_PROTO_HDRS ${mav_msgs})
PROTOBUF_GENERATE_CPP(NAV_PROTO_SRCS NAV_PROTO_HDRS ${nav_msgs})
PROTOBUF_GENERATE_CPP(PHY_PROTO_SRCS PHY_PROTO_HDRS ${physics_msgs})
PROTOBUF_GENERATE_CPP(STD_PROTO_SRCS STD_PROTO_HDRS ${std_msgs})
PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})

generate_protobuf_msgs(mav_msgs ${MAV_PROTO_SRCS})
generate_protobuf_msgs(nav_msgs ${NAV_PROTO_SRCS})
generate_protobuf_msgs(physics_msgs ${PHY_PROTO_SRCS})
generate_protobuf_msgs(std_msgs ${STD_PROTO_SRCS})
generate_protobuf_msgs(sensor_msgs ${SEN_PROTO_SRCS})

#---------#
# Plugins #
#---------#

add_plugin(gazebo_geotagged_images_plugin src/gazebo_geotagged_images_plugin.cpp)
add_plugin(gazebo_gps_plugin src/gazebo_gps_plugin.cpp)
add_plugin(gazebo_irlock_plugin src/gazebo_irlock_plugin.cpp)
add_plugin(gazebo_lidar_plugin src/gazebo_lidar_plugin.cpp)
add_plugin(gazebo_sonar_plugin src/gazebo_sonar_plugin.cpp)
add_plugin(gazebo_uuv_plugin src/gazebo_uuv_plugin.cpp)
add_plugin(gazebo_vision_plugin src/gazebo_vision_plugin.cpp)
add_plugin(gazebo_controller_interface src/gazebo_controller_interface.cpp)
add_plugin(gazebo_gimbal_controller_plugin src/gazebo_gimbal_controller_plugin.cpp)
add_plugin(gazebo_imu_plugin src/gazebo_imu_plugin.cpp)
add_plugin(gazebo_mavlink_interface src/gazebo_mavlink_interface.cpp )
add_plugin(gazebo_motor_model src/gazebo_motor_model.cpp)
add_plugin(gazebo_multirotor_base_plugin src/gazebo_multirotor_base_plugin.cpp)
add_plugin(gazebo_wind_plugin src/gazebo_wind_plugin.cpp)
add_plugin(gazebo_magnetometer_plugin src/gazebo_magnetometer_plugin.cpp src/geo_mag_declination.cpp)
add_plugin(gazebo_barometer_plugin src/gazebo_barometer_plugin.cpp)
add_plugin(gazebo_motor_failure_plugin src/gazebo_motor_failure_plugin.cpp)
add_plugin(gazebo_lift_drag_plugin src/liftdrag_plugin/liftdrag_plugin.cpp)


set(plugins
  gazebo_geotagged_images_plugin
  gazebo_gps_plugin
  gazebo_irlock_plugin
  gazebo_lidar_plugin
  gazebo_sonar_plugin
  gazebo_uuv_plugin
  gazebo_vision_plugin
  gazebo_controller_interface
  gazebo_gimbal_controller_plugin
  gazebo_imu_plugin
  gazebo_mavlink_interface
  gazebo_motor_model
  gazebo_multirotor_base_plugin
  gazebo_wind_plugin
  gazebo_magnetometer_plugin
  gazebo_barometer_plugin
  gazebo_motor_failure_plugin
  gazebo_lift_drag_plugin
  )

#foreach(plugin ${plugins})
#  target_link_libraries(${plugin} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${TinyXML_LIBRARIES})
#endforeach()
#target_link_libraries(gazebo_opticalflow_plugin ${OpticalFlow_LIBS})

# If BUILD_ROS_INTERFACE set to ON, build plugins that have ROS dependencies
# Current plugins that can be used with ROS interface: gazebo_motor_failure_plugin
# if (BUILD_ROS_INTERFACE)
#   add_library(gazebo_motor_failure_plugin SHARED src/gazebo_motor_failure_plugin.cpp)
#   target_link_libraries(gazebo_motor_failure_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})
#   list(APPEND plugins gazebo_motor_failure_plugin)
#   message(STATUS "adding gazebo_motor_failure_plugin to build")

#   include_directories(
#     include
#     ${geometry_msgs_INCLUDE_DIRS}
#     ${sensor_msgs_INCLUDE_DIRS}
#   )

#   target_link_libraries(gazebo_motor_failure_plugin
#     ${catkin_LIBRARIES}
#     ${roscpp_LIBRARIES}
#     ${GAZEBO_libraries}
#   )
# endif()

# Linux is not consistent with plugin availability, even on Gazebo 7
# #if("${GAZEBO_VERSION}" VERSION_LESS "7.0")
#   add_library(LiftDragPlugin SHARED src/liftdrag_plugin/liftdrag_plugin.cpp)
#   list(APPEND plugins LiftDragPlugin)
# #endif()

# foreach(plugin ${plugins})
  # add_dependencies(${plugin} mav_msgs nav_msgs std_msgs sensor_msgs)
  # add_dependencies(${plugin} physics_msgs)
# endforeach()

# Configure the setup script
if (catkin_FOUND)
  catkin_add_env_hooks(50_sitl_gazebo_setup
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
    SHELLS sh)
endif()


################
## Unit Tests ##
################

include(UnitTests)

add_subdirectory(unit_tests)


#############
## Install ##
#############

set(PLUGIN_PATH ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/plugins)
set(MODEL_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/models)
set(RESOURCE_PATH ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

# file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/models/.DS_Store)
# file(GLOB models_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/models/*)

# file(REMOVE_RECURSE ${PROJECT_SOURCE_DIR}/worlds/.DS_Store)
# file(GLOB worlds_list LIST_DIRECTORIES true ${PROJECT_SOURCE_DIR}/worlds/*)

# install(TARGETS ${plugins} mav_msgs nav_msgs std_msgs sensor_msgs DESTINATION ${PLUGIN_PATH})
# install(DIRECTORY ${models_list} DESTINATION ${MODEL_PATH})
# install(FILES ${worlds_list} DESTINATION ${RESOURCE_PATH}/worlds)

configure_file(src/setup.sh.in "${CMAKE_CURRENT_BINARY_DIR}/setup.sh" @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION ${RESOURCE_PATH})

install(FILES ${PROJECT_SOURCE_DIR}/package.xml DESTINATION ${RESOURCE_PATH})

#############
## Testing ##
#############

# TODO

###############
## Packaging ##
###############

set(CPACK_PACKAGE_NAME ${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_CONTACT pxusers@googlegroups.com)
set(DEBIAN_PACKAGE_DEPENDS "")
set(RPM_PACKAGE_DEPENDS "")

set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEBIAN_PACKAGE_DEPENDS})
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "gazebo plugins for px4 sitl.")

set(CPACK_RPM_PACKAGE_REQUIRES "${DEBIAN_PACKAGE_DEPENDS}")
set(CPACK_RPM_PACKAGE_DESCRIPTION "Gazebo plugins for px4 sitl.")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${GAZEBO_MAJOR_VERSION}-${PROJECT_VERSION}")

include(CPack)
