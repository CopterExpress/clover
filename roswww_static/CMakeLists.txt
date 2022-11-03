cmake_minimum_required(VERSION 3.0)
project(roswww_static)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS update
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

message(status "CATKIN_ENV: ${CATKIN_ENV}")

add_custom_target(roswww_static ALL
    COMMAND ${CATKIN_ENV} ${CMAKE_CURRENT_SOURCE_DIR}/update)
