cmake_minimum_required(VERSION 2.8.3)
project(mobile_base_driver)

find_package(catkin REQUIRED COMPONENTS
   actionlib_msgs
   roscpp
   rospy
   std_msgs
   message_generation
)

add_message_files(
  FILES
  BatteryCapacity.msg
  Bumper.msg
  ChestLeds.msg
  CliffArray.msg
  CliffSensor.msg
  Clothesline.msg
  Led.msg
  Motor.msg
  Power.msg
  SafetyStatus.msg
  Sensors.msg
  Stall.msg
  Telescope.msg
  Touch.msg
  WheelDrop.msg
  WheelTraj.msg
  WheelTrajPoint.msg
)

add_action_files(
  DIRECTORY action
  FILES ArcMove.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS actionlib_msgs message_runtime std_msgs)

