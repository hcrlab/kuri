cmake_minimum_required(VERSION 2.8.3)
project(mobile_base_driver)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

add_message_files(
  FILES
  Bumper.msg
  ChestLeds.msg
  CliffArray.msg
  CliffSensor.msg
  Led.msg
  Sensors.msg
  Touch.msg
  WheelDrop.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

