cmake_minimum_required(VERSION 3.8)
project(waypoint_manager VERSION 1.5.0)

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
find_package(uav_interfaces REQUIRED)

add_executable(waypoint_manager_node src/waypoint_manager_node.cpp)
target_include_directories(waypoint_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(waypoint_manager_node
  geometry_msgs
  rclcpp
  std_msgs
  uav_interfaces)

add_executable(competition_mission_node src/competition_mission_node.cpp)
target_include_directories(competition_mission_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(competition_mission_node
  geometry_msgs
  rclcpp
  std_msgs
  uav_interfaces)

add_executable(
  competition_flight_directive_node
  src/competition_flight_directive_node.cpp)
target_include_directories(competition_flight_directive_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(competition_flight_directive_node
  rclcpp std_msgs uav_interfaces)

install(TARGETS waypoint_manager_node competition_mission_node competition_flight_directive_node
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

  # 验证到达确认、停留计时以及短时波动容忍。
  ament_add_gtest(test_arrival_tracker test/test_arrival_tracker.cpp)
  if(TARGET test_arrival_tracker)
    target_include_directories(test_arrival_tracker PRIVATE include)
  endif()

  # 验证航线字段检查、航向归一化和非法航线拒绝。
  ament_add_gtest(test_route_validation test/test_route_validation.cpp)
  if(TARGET test_route_validation)
    target_include_directories(test_route_validation PRIVATE include)
    ament_target_dependencies(test_route_validation uav_interfaces)
  endif()

  ament_add_gtest(test_competition_geometry test/test_competition_geometry.cpp)
  if(TARGET test_competition_geometry)
    target_include_directories(test_competition_geometry PRIVATE include)
  endif()

  # 启动真实节点，检查“等待地面站航线”和完整单航点任务流程。
  add_launch_test(test/test_waypoint_manager_launch.py TIMEOUT 60)
  add_launch_test(test/test_competition_mission_launch.py TIMEOUT 60)
  add_launch_test(test/test_competition_task2_launch.py TIMEOUT 60)
  add_launch_test(test/test_competition_flight_directive_launch.py TIMEOUT 60)
endif()

ament_package()
