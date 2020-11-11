cmake_minimum_required(VERSION 2.8.3)
project(kuri_web_teleop)

set(OpenCV_DIR /usr/share/OpenCV/)


add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  roslib
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  mobile_base_driver
  joy
  actionlib
  image_transport
  compressed_image_transport
  image_geometry
  tf
  web_video_server
)

catkin_package(
    CATKIN_DEPENDS
    mobile_base_driver
    tf
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(add_image_overlay src/add_image_overlay.cpp)

target_link_libraries(add_image_overlay ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


install(TARGETS add_image_overlay
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
