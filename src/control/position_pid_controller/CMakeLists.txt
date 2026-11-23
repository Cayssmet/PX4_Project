cmake_minimum_required(VERSION 3.8)
project(position_pid_controller VERSION 1.2.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(position_pid_node src/position_pid_node.cpp)
target_include_directories(position_pid_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(position_pid_node
  geometry_msgs
  rclcpp
  std_msgs
  tf2)

install(TARGETS position_pid_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_pid_axis test/test_pid_axis.cpp)
  if(TARGET test_pid_axis)
    target_include_directories(test_pid_axis PRIVATE include)
  endif()

  ament_add_gtest(test_control_math test/test_control_math.cpp)
  if(TARGET test_control_math)
    target_include_directories(test_control_math PRIVATE include)
  endif()

  # 启动真实节点，验证话题覆盖目标以及无效/过期输入时的零速度保护。
  add_launch_test(test/test_position_pid_launch.py TIMEOUT 60)
endif()

ament_package()
