cmake_minimum_required(VERSION 3.8)
project(uav_interfaces VERSION 1.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 列出本包生成的全部消息和服务。
# 后续若新增 .msg 或 .srv 文件，必须同时把文件路径加入这里并重新构建。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/WaypointList.msg"
  "msg/FlightDirective.msg"
  "msg/FlightCommand.msg"
  "msg/FlightStatus.msg"
  "msg/MissionStatus.msg"
  "msg/CarStatus.msg"
  "msg/MissionSync.msg"
  "msg/UavStatus.msg"
  "msg/GcsStatus.msg"
  "srv/SetWaypointList.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

# 下游功能包在运行时需要加载生成的类型支持库。
ament_export_dependencies(rosidl_default_runtime)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 这个测试会实际包含生成后的 C++ 头文件，能够发现接口没有正确生成或字段不一致。
  ament_add_gtest(test_generated_interfaces test/test_generated_interfaces.cpp)
  if(TARGET test_generated_interfaces)
    rosidl_get_typesupport_target(cpp_typesupport_target
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(test_generated_interfaces "${cpp_typesupport_target}")
  endif()
endif()

ament_package()
