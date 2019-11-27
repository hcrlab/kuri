cmake_minimum_required(VERSION 2.8.3)
project(kuri_camera)
set(OpenCV_DIR /usr/share/OpenCV/)


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
  INCLUDE_DIRS include
  CATKIN_DEPENDS
        cv_bridge
        image_transport
        sensor_msgs
)

set(libavcodec "libavcodec-NOTFOUND")
find_file(libavcodec libavcodec.so /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
message(WARNING "libavcodec path: ")
message(WARNING ${libavcodec})
set(libavutil "libavutil-NOTFOUND")
find_file(libavutil libavutil.so /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
message(WARNING "libavutil path: ")
message(WARNING ${libavutil})
set(libswscale "libswscale-NOTFOUND")
find_file(libswscale libswscale.so /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
message(WARNING "libswscale path: ")
message(WARNING ${libswscale})
add_library(h264Decoder src/h264_decoder.cpp)
target_link_libraries(h264Decoder ${libavcodec} ${libavutil} ${libswscale})


include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# set(libmadmux "libmadmux-NOTFOUND")
# find_file(libmadmux libmadmux.so.0 /opt/gizmo-master-cbb8e0c/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
# find_file(libmadmux libmadmux.so.0 /opt/ros/indigo/lib NO_DEFAULT_PATH)
# message(WARNING "libmadmux path: ")
# message(WARNING ${libmadmux})
# add_executable(ros_publisher src/ros_publisher.cpp)
# target_link_libraries(ros_publisher  ${catkin_LIBRARIES} ${libmadmux})
# add_executable(ros_publisher_test src/ros_publisher_test.cpp)
# target_link_libraries(ros_publisher_test  ${catkin_LIBRARIES} ${libmadmux})


add_executable(udsToTCP src/udsToTCP.cpp)
target_link_libraries(udsToTCP ${catkin_LIBRARIES})
add_executable(tcpClient src/tcpClient.cpp)
target_link_libraries(tcpClient ${catkin_LIBRARIES})
add_executable(h264DecoderNode src/h264_decoder_node.cpp)
target_link_libraries(h264DecoderNode ${catkin_LIBRARIES} h264Decoder)
#
#add_executable(save_image src/save_image.cpp)
#target_link_libraries(save_image ${catkin_LIBRARIES} ${libmadmux})
#
# install(TARGETS h264Decoder
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )
#
#install(TARGETS save_image
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )
#
#endif()
