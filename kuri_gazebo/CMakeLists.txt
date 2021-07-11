cmake_minimum_required(VERSION 2.8.3)
project(kuri_gazebo)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    control_toolbox
    gazebo_ros_control
    hardware_interface
    joint_limits_interface
)

find_package(gazebo REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
        control_toolbox
        gazebo_ros_control
        joint_limits_interface
        hardware_interface
)


include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/kuri_hardware_gazebo.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
install (FILES kuri_gazebo_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
