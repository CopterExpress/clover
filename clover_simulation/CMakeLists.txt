cmake_minimum_required(VERSION 3.0)
project(clover_simulation)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Gazebo LED strip node

find_package(GAZEBO)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  led_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(led_strip src/led_strip.cpp)

target_link_libraries(led_strip ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
