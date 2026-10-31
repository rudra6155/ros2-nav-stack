cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# Worlds live at the repository root (worlds/) and are installed next to the launch files.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  FILES_MATCHING PATTERN "*.world" PATTERN "*.obstacles" PATTERN "world_stats.json")

ament_package()
