cmake_minimum_required(VERSION 3.8)
project(control)

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
find_package(tf2_ros REQUIRED)
find_package(navigation REQUIRED)

add_executable(waypoint_follower src/waypoint_follower.cpp)
target_include_directories(waypoint_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(waypoint_follower navigation::navigation_core)
ament_target_dependencies(waypoint_follower rclcpp geometry_msgs nav_msgs tf2_ros)

install(TARGETS waypoint_follower DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_msgs REQUIRED)
  # Node-in-the-loop test: the real WaypointFollower node against a lagged simulated plant.
  # Own ROS_DOMAIN_ID so it cannot talk to anything else running on the machine.
  ament_add_gtest(test_waypoint_follower test/test_waypoint_follower.cpp
    TIMEOUT 60 ENV ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1)
  target_include_directories(test_waypoint_follower PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_waypoint_follower navigation::navigation_core)
  ament_target_dependencies(test_waypoint_follower rclcpp geometry_msgs nav_msgs tf2_ros tf2_msgs)
endif()

ament_package()
