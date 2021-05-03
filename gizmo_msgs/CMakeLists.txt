cmake_minimum_required(VERSION 2.8.3)
project(gizmo_msgs)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED
        COMPONENTS
        actionlib
        actionlib_msgs
        audio_msgs
        geometry_msgs
        mayfield_msgs
        message_generation
        mobile_base_driver
        nav_msgs
        sensor_msgs
        std_msgs
        )

add_action_files(
        FILES
        CalibrationPath.action
        RecordBagfile.action
)

add_message_files(
        FILES
        Action.msg
        Asleep.msg
        Awake.msg
        Bumpers.msg
        ChestLED.msg
        ClientTouch.msg
        ClientWaypoint.msg
        Command.msg
        CompressedOccupancyGrid.msg
        Connection.msg
        Deaf.msg
        Exchange.msg
        Field.msg
        FilteredBattery.msg
        HeadPose.msg
        MomentAction.msg
        MomentActions.msg
        MqttMessage.msg
        NavigationFeedback.msg
        NavigationGoal.msg
        RomojiFeedback.msg
        RomojiGoal.msg
        SoundHoundCommand.msg
        StringStamped.msg
        VolumeButton.msg
        Volume.msg
)

add_service_files(
        FILES
        Deafen.srv
        GetDirection.srv
        GetField.srv
        ListFields.srv
        MqttStatus.srv
        MqttSubscribe.srv
        PhotoCapture.srv
        RobotInfo.srv
        RobotPose.srv
        SetField.srv
        SetNavMode.srv
        Snooze.srv
        Stat.srv
        UploaderStatus.srv
        WakeUp.srv

)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        audio_msgs
        geometry_msgs
        mayfield_msgs
        mobile_base_driver
        nav_msgs
        sensor_msgs
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        actionlib_msgs
        audio_msgs
        geometry_msgs
        nav_msgs
        mayfield_msgs
        mobile_base_driver
        sensor_msgs
        std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
