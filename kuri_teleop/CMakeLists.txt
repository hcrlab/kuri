cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(kuri_teleop)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  joy
  actionlib
  actionlib_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs trajectory_msgs control_msgs joy actionlib actionlib_msgs tf
)

catkin_add_env_hooks(${PROJECT_NAME} SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joystick_teleop src/joystick_teleop.cpp)
target_link_libraries(joystick_teleop ${catkin_LIBRARIES})

add_executable(display_heading_arrow src/display_heading_arrow.cpp)
target_link_libraries(display_heading_arrow ${catkin_LIBRARIES})


install(TARGETS joystick_teleop display_heading_arrow
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/keyboard_teleop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
       
       
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
