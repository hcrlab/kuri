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


include_directories(${catkin_INCLUDE_DIRS})
