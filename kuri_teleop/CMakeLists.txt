cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(kuri_teleop)

find_package(
  catkin REQUIRED
  roscpp
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  joy
  actionlib
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs trajectory_msgs control_msgs joy actionlib tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joystick_teleop src/joystick_teleop.cpp)
target_link_libraries(joystick_teleop ${catkin_LIBRARIES})

add_executable(display_heading_arrow src/display_heading_arrow.cpp)
target_link_libraries(display_heading_arrow ${catkin_LIBRARIES})
