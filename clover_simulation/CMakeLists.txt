cmake_minimum_required(VERSION 3.0)
project(clover_simulation)

find_package(catkin REQUIRED)

# Gazebo LED strip node

find_package(gazebo)

# Gazebo does not seem to export GAZEBO_FOUND, so here's a matching hack
if (NOT "${GAZEBO_CONFIG_INCLUDED}")
  message(STATUS "Gazebo not found, skipping")
  return()
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  led_msgs
  gazebo_ros
  gazebo_plugins
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS led_msgs gazebo_ros gazebo_plugins
)

# Gazebo LED plugin

add_library(sim_leds src/sim_leds.cpp)

target_include_directories(sim_leds PRIVATE ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(sim_leds PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_compile_options(sim_leds PRIVATE -std=c++11)

add_dependencies(sim_leds ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(sim_leds_controller src/sim_leds_controller.cpp)

target_include_directories(sim_leds_controller PRIVATE ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(sim_leds_controller PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_compile_options(sim_leds_controller PRIVATE -std=c++11)

add_dependencies(sim_leds_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Gazebo throttling camera plugin
# for some reason, CMake does not support per-target link directories, and Gazebo does not put
# CameraPlugin into ${GAZEBO_LIBRARIES}

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(throttling_camera src/throttling_camera.cpp)

target_include_directories(throttling_camera PRIVATE ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(throttling_camera PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} CameraPlugin)
target_compile_options(throttling_camera PRIVATE -std=c++11)

add_dependencies(throttling_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/aruco_gen
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
