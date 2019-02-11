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

find_library(libmadmux NAMES libmadmux)

catkin_package(
  CATKIN_DEPENDS
        cv_bridge
        image_transport
        sensor_msgs
)


include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# TODO: Figure out why we can't seem to link against madmux on the robot
#if(libmadmux)
#    message(WARNING "madmux wasn't found. Disabling C++ targets")
#else()
#add_executable(ros_publisher src/ros_publisher.cpp)
#target_link_libraries(ros_publisher ${catkin_LIBRARIES} ${libmadmux})
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

