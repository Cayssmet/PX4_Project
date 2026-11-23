cmake_minimum_required(VERSION 3.8)
project(camera_serial_bridge VERSION 1.0.0)

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

add_library(${PROJECT_NAME}_core src/protocol.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(camera_serial_bridge_node src/camera_serial_bridge_node.cpp)
target_include_directories(camera_serial_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camera_serial_bridge_node ${PROJECT_NAME}_core)
ament_target_dependencies(camera_serial_bridge_node geometry_msgs rclcpp std_msgs)

install(TARGETS ${PROJECT_NAME}_core camera_serial_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_camera_protocol test/test_camera_protocol.cpp)
  if(TARGET test_camera_protocol)
    target_include_directories(test_camera_protocol PRIVATE include)
    target_link_libraries(test_camera_protocol ${PROJECT_NAME}_core)
  endif()
  add_launch_test(test/test_camera_bridge_launch.py TIMEOUT 45)
endif()
ament_package()
