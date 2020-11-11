cmake_minimum_required(VERSION 2.8.3)
project(kuri_navigation)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED
  COMPONENTS
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS

)

catkin_install_python(PROGRAMS scripts/mapping_controller scripts/may_nav_controller scripts/safety_controller
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
