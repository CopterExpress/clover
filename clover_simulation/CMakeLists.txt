cmake_minimum_required(VERSION 3.0)
project(clover_simulation)

find_package(catkin REQUIRED)

# Gazebo LED strip node

find_package(gazebo REQUIRED)
#include_directories(${GAZEBO_INCLUDE_DIRS})
#link_directories(${GAZEBO_LIBRARY_DIRS})
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  led_msgs
  gazebo_ros
)

catkin_package(
  CATKIN_DEPENDS led_msgs gazebo_ros
)

add_executable(led_strip src/led_strip.cpp)
target_include_directories(led_strip PRIVATE ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(led_strip ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_compile_options(led_strip PRIVATE -std=c++11)

add_dependencies(led_strip ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Gazebo LED plugin

add_library(sim_leds src/sim_leds.cpp)

target_include_directories(sim_leds PRIVATE ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(sim_leds PRIVATE ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_compile_options(sim_leds PRIVATE -std=c++11)

add_dependencies(sim_leds ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
