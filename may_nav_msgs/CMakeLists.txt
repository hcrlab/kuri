cmake_minimum_required(VERSION 2.8.3)
project(may_nav_msgs)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED
        COMPONENTS
        actionlib_msgs
        geometry_msgs
        message_generation
        nav_msgs
        std_msgs
        )

add_action_files(
        FILES
        Navigate.action
)

add_message_files(
        FILES
        CostMap.msg
        ScoredTrajectories.msg
        Trajectory.msg
)

add_service_files(
        FILES
        GetCostMap.srv
        GetObstacleDist.srv
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        geometry_msgs
        nav_msgs
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
