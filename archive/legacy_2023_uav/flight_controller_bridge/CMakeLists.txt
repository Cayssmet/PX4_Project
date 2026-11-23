cmake_minimum_required(VERSION 3.8)
project(flight_controller_bridge VERSION 1.1.0)

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

# 协议编解码和坐标变换不依赖 ROS 节点，单独做成库，便于单元测试和后续复用。
add_library(${PROJECT_NAME}_core
  src/protocol.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(flight_controller_bridge_node
  src/flight_controller_bridge_node.cpp)
target_include_directories(flight_controller_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flight_controller_bridge_node ${PROJECT_NAME}_core)
ament_target_dependencies(flight_controller_bridge_node
  geometry_msgs
  rclcpp
  std_msgs
  uav_interfaces)

install(TARGETS
  ${PROJECT_NAME}_core
  flight_controller_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config docs launch
  DESTINATION share/${PROJECT_NAME})

# 无需接飞控硬件即可人工联调的伪飞控；仅使用 Python 标准库。
install(PROGRAMS
  tools/fake_flight_controller.py
  tools/fake_flight_controller_v2.py
  tools/manual_v2_acceptance_publisher.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # 纯 C++ 测试：CRC、定长帧、抗错位解析、坐标旋转和定点数换算。
  ament_add_gtest(test_protocol_and_coordinates
    test/test_protocol_and_coordinates.cpp)
  if(TARGET test_protocol_and_coordinates)
    target_include_directories(test_protocol_and_coordinates PRIVATE include)
    target_link_libraries(test_protocol_and_coordinates ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_protocol_v2 test/test_protocol_v2.cpp)
  if(TARGET test_protocol_v2)
    target_include_directories(test_protocol_v2 PRIVATE include)
    target_link_libraries(test_protocol_v2 ${PROJECT_NAME}_core)
  endif()

  # 使用 Linux 伪终端模拟一根真实串口线，验证 ROS 话题到线缆帧的完整链路。
  add_launch_test(test/test_bridge_launch.py TIMEOUT 60)
  add_launch_test(test/test_bridge_v2_launch.py TIMEOUT 60)
endif()

ament_package()
