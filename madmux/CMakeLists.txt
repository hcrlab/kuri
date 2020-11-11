cmake_minimum_required(VERSION 2.8.3)
project(madmux)

find_package(catkin REQUIRED)

catkin_package(
        INCLUDE_DIRS include
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})