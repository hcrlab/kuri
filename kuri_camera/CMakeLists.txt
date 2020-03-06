cmake_minimum_required(VERSION 2.8.3)
project(kuri_camera)
# NOTE: You may have to change the following paths
set(libmadmux_path /opt/gizmo/lib/x86_64-linux-gnu)
set(OpenCV_DIR /usr/share/OpenCV/)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
        cv_bridge
        image_transport
        madmux
        compressed_image_transport
        dynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
        cv_bridge
        image_transport
        sensor_msgs
        compressed_image_transport
        dynamic_reconfigure
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Uncomment this if you would like to use the camera's h264 encoding
# # find libavcodec
# set(libavcodec "libavcodec-NOTFOUND")
# find_library(libavcodec libavcodec.so)
# # find libavutil
# set(libavutil "libavutil-NOTFOUND")
# find_library(libavutil libavutil.so)
# # find libswscale
# set(libswscale "libswscale-NOTFOUND")
# find_library(libswscale libswscale.so)
# if (libavcodec AND libavutil AND libswscale)
#   message(STATUS "Found libavcodec, libavutil, and libswscale. Building h264_decoder_node")
#   add_library(h264_decoder src/h264_decoder.cpp)
#   target_link_libraries(h264_decoder ${libavcodec} ${libavutil} ${libswscale} ${OpenCV_LIBRARIES})
#   add_executable(h264_decoder_node src/h264_decoder_node.cpp)
#   target_link_libraries(h264_decoder_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} h264_decoder)
# else()
#   message(WARNING "Did not find libavcodec, libavutil, and/or libswscale. Skipping h264_decoder_node")
#   message(WARNING "libavcodec path: ")
#   message(WARNING ${libavcodec})
#   message(WARNING "libavutil path: ")
#   message(WARNING ${libavutil})
#   message(WARNING "libswscale path: ")
#   message(WARNING ${libswscale})
# endif()

# find libmadmux
set(libmadmux "libmadmux-NOTFOUND")
find_file(libmadmux libmadmux.so.0 ${libmadmux_path} NO_DEFAULT_PATH)
if (libmadmux)
  message(STATUS "Found libmadmux. Building kuri_camera_publisher")
  add_executable(kuri_camera_publisher src/kuri_camera_publisher.cpp)
  add_dependencies(kuri_camera_publisher KuriCamera_gencfg)
  target_link_libraries(kuri_camera_publisher  ${catkin_LIBRARIES} ${libmadmux})
else()
  message(WARNING "Did not find libmadmux. Skipping kuri_camera_publisher")
  message(WARNING "libmadmux path: ")
  message(WARNING ${libmadmux})
endif()

if (libavcodec AND libavutil AND libswscale AND libmadmux)
  message(STATUS "Found everything. Building h264_decoder_node_on_kuri")
  #add_library(h264_decoder_on_kuri src/h264_decoder_on_kuri.cpp)
  #target_link_libraries(h264_decoder_on_kuri ${libavcodec} ${libavutil} ${libswscale} ${OpenCV_LIBRARIES} ${libmadmux})
  #add_executable(h264_decoder_node_on_kuri src/h264_decoder_node_on_kuri.cpp)
  #target_link_libraries(h264_decoder_node_on_kuri ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} h264_decoder_on_kuri)
else()
  message(WARNING "Did not find libavcodec, libavutil, libswscale, and/or libmadmux. Skipping h264_decoder_node_on_kuri")
  message(WARNING "libavcodec path: ")
  message(WARNING ${libavcodec})
  message(WARNING "libavutil path: ")
  message(WARNING ${libavutil})
  message(WARNING "libswscale path: ")
  message(WARNING ${libswscale})
  message(WARNING "libmadmux path: ")
  message(WARNING ${libmadmux})
endif()

#add_executable(uds_to_tcp src/uds_to_tcp.cpp)
#target_link_libraries(uds_to_tcp ${catkin_LIBRARIES})
#add_executable(tcp_test_client src/tcp_test_client.cpp)
#target_link_libraries(tcp_test_client ${catkin_LIBRARIES})
add_executable(display_compressed_images src/display_compressed_images.cpp)
target_link_libraries(display_compressed_images ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(kuri_camera_subscriber src/kuri_camera_subscriber.cpp)
target_link_libraries(kuri_camera_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
