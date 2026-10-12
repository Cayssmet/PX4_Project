cmake_minimum_required(VERSION 3.8)
project(px4_offboard VERSION 2.0.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 坐标换算是纯数学，不依赖 ROS 也不依赖 Eigen，单独做成库：
# 这样它能脱离 ROS 环境编译，单元测试也不受中间件影响。
# 注意：qos_profiles.hpp 是 header-only、需要 rclcpp，因此它只走 include/ 安装，
# 不进这个库。
add_library(${PROJECT_NAME}_core
  src/frame_transforms.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 坐标系换算是本项目炸机风险第一名，这一步测试必须独立通过：
  #   colcon test --packages-select px4_offboard --event-handlers console_direct+
  ament_add_gtest(test_frame_transforms
    test/test_frame_transforms.cpp)
  if(TARGET test_frame_transforms)
    target_link_libraries(test_frame_transforms ${PROJECT_NAME}_core)
  endif()
endif()

ament_package()
