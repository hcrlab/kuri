cmake_minimum_required(VERSION 2.8.3)
project(kuri_api)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs

)
catkin_python_setup()

add_message_files(
  FILES
  Volume.msg
  VolumeButton.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

catkin_install_python(PROGRAMS scripts/base_demo scripts/volume_interface
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

