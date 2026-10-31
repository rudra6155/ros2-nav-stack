cmake_minimum_required(VERSION 3.8)
project(evaluation)

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
find_package(std_srvs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(navigation REQUIRED)

add_executable(run_episodes src/run_episodes.cpp)
target_include_directories(run_episodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(run_episodes navigation::navigation_core)
ament_target_dependencies(run_episodes
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs gazebo_msgs ament_index_cpp)

add_executable(controller_check src/controller_check.cpp)
target_include_directories(controller_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(controller_check navigation::navigation_core)
ament_target_dependencies(controller_check rclcpp geometry_msgs nav_msgs gazebo_msgs)

install(TARGETS run_episodes controller_check DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/summarize.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_episode_sampler test/test_episode_sampler.cpp)
  target_include_directories(test_episode_sampler PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_episode_sampler navigation::navigation_core)
  target_compile_definitions(test_episode_sampler PRIVATE
    WORLDS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../worlds"
    GOLDEN_SIMPLE_1007_START_X=-5.0589271354447138
    GOLDEN_SIMPLE_1007_START_Y=-1.5565992480438293
    GOLDEN_SIMPLE_1007_START_YAW=1.5787599392881928
    GOLDEN_SIMPLE_1007_GOAL_X=3.0225525559417186
    GOLDEN_SIMPLE_1007_GOAL_Y=-1.6364364615547888)

  ament_add_pytest_test(test_summarize test/test_summarize.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
endif()

ament_package()
