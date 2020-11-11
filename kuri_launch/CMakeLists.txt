cmake_minimum_required(VERSION 2.8.3)
project(kuri_launch)

find_package(catkin REQUIRED)

catkin_add_env_hooks(${PROJECT_NAME} SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)


catkin_package(
)

