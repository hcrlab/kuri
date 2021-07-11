cmake_minimum_required(VERSION 2.8.3)
project(kuri_slack_bot)


find_package(catkin REQUIRED)

catkin_package(
)

catkin_install_python(PROGRAMS dummy_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
