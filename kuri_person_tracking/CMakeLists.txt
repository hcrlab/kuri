cmake_minimum_required(VERSION 2.8.3)
project(kuri_person_tracking)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)


catkin_install_python(PROGRAMS scripts/person_detection.py scripts/move_towards.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/download_yolo.bash
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts)
