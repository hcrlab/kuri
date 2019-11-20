cmake_minimum_required(VERSION 2.8.3)
project(kuri_camera)


add_compile_options(-std=c++11)

find_package( OpenCV REQUIRED)


find_package(catkin REQUIRED
  COMPONENTS
        cv_bridge
        image_transport
        madmux

)

#find_library(libmadmux NAMES libmadmux)

catkin_package(
  CATKIN_DEPENDS
        cv_bridge
        image_transport
        sensor_msgs
)


include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(libmadmux "libmadmux-NOTFOUND")
find_file(libmadmux libmadmux.so.0 /opt/gizmo-master-cbb8e0c/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
#find_file(libmadmux libmadmux.so.0 /opt/ros/indigo/lib NO_DEFAULT_PATH)
message(WARNING "libmadmux path: ")
message(WARNING ${libmadmux})
add_executable(ros_publisher src/ros_publisher.cpp)
target_link_libraries(ros_publisher  ${catkin_LIBRARIES} ${libmadmux})
add_executable(ros_publisher_test src/ros_publisher_test.cpp)
target_link_libraries(ros_publisher_test  ${catkin_LIBRARIES} ${libmadmux})
#
#add_executable(save_image src/save_image.cpp)
#target_link_libraries(save_image ${catkin_LIBRARIES} ${libmadmux})
#
#install(TARGETS ros_publisher
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )
#
#install(TARGETS save_image
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )
#
#endif()

