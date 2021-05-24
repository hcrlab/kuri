cmake_minimum_required(VERSION 2.8.3)
project(kuri_navigation)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED
        COMPONENTS
        geometry_msgs
        mobile_base_driver
        pcl_conversions
        pcl_ros
        roscpp
        rospy
        )

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS
        geometry_msgs
        roscpp
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(bumper_to_pointcloud
        src/bumper_to_pointcloud.cpp
        )


add_dependencies(bumper_to_pointcloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bumper_to_pointcloud
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )

install(
        TARGETS
        bumper_to_pointcloud
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

catkin_install_python(PROGRAMS scripts/mapping_controller scripts/may_nav_controller scripts/safety_controller
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
