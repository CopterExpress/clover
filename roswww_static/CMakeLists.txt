cmake_minimum_required(VERSION 3.0)
project(roswww_static)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS main.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
