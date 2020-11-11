cmake_minimum_required(VERSION 2.8.3)
project(kuri_audio)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED
        COMPONENTS
        rospy
        )

catkin_package(
        CATKIN_DEPENDS
        

)

catkin_install_python(PROGRAMS scripts/audio_voice_delegate
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
