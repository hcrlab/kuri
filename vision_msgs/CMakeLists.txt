cmake_minimum_required(VERSION 2.8.3)
project(vision_msgs)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED
  COMPONENTS
        geometry_msgs
        mayfield_msgs
        message_generation
        sensor_msgs
        )

add_message_files(
        FILES
        ClassifiedObject.msg
        ClassifiedObjects.msg
        DetectedObjects.msg
        Face.msg
        FaceArray.msg
        FrameResults.msg
        ImageClustering.msg
        ImageQuality.msg
        ObjectFeatures.msg
        VisionCmdMsg.msg
)

add_service_files(
        FILES
        VisionActiveModules.srv
        VisionCmds.srv
        VisionQuery.srv
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        sensor_msgs
        std_msgs
        mayfield_msgs
)

catkin_package(
        CATKIN_DEPENDS
        geometry_msgs
        sensor_msgs
        std_msgs
        mayfield_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
