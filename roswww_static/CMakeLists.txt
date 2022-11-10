cmake_minimum_required(VERSION 3.0)
project(roswww_static)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS src/update
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
