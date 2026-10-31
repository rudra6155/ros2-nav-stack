cmake_minimum_required(VERSION 3.8)
project(navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ROS-free algorithms: A* and pure pursuit (also used by control/ and evaluation/)
add_library(navigation_core src/astar.cpp src/pure_pursuit.cpp)
target_include_directories(navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(astar_planner src/astar_planner.cpp)
target_link_libraries(astar_planner navigation_core)
ament_target_dependencies(astar_planner rclcpp geometry_msgs nav_msgs std_msgs tf2_ros)

install(TARGETS navigation_core EXPORT export_navigation_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS astar_planner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_astar test/test_astar.cpp)
  target_link_libraries(test_astar navigation_core)
  ament_add_gtest(test_pure_pursuit test/test_pure_pursuit.cpp)
  target_link_libraries(test_pure_pursuit navigation_core)
endif()

ament_export_targets(export_navigation_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
