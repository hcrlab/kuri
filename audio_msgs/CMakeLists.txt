cmake_minimum_required(VERSION 2.8.3)
project(audio_msgs)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED
        COMPONENTS
        geometry_msgs
        mayfield_msgs
        message_generation
        std_msgs
        )

add_message_files(
        FILES
        Asleep.msg
        Awake.msg
        Deaf.msg
        Exchange.msg
        Field.msg
        SoundHoundCommand.msg
)

add_service_files(
        FILES
        Deafen.srv
        GetDirection.srv
        GetField.srv
        ListFields.srv
        SetField.srv
        Snooze.srv
        Stat.srv
        WakeUp.srv
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        mayfield_msgs
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
