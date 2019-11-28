cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(kuri_joystick_teleop)

find_package(
  catkin REQUIRED
  roscpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  joy
  actionlib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs trajectory_msgs control_msgs joy actionlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(kuri_joystick_teleop src/kuri_joystick_teleop.cpp)

target_link_libraries(kuri_joystick_teleop ${catkin_LIBRARIES})
