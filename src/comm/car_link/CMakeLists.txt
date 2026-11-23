cmake_minimum_required(VERSION 3.8)
project(car_link VERSION 0.1.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uav_interfaces REQUIRED)

# v0.1.0 只有目录骨架，节点还没写。
# 节点写好后在这里加：
#   add_executable(car_link_node src/car_link_node.cpp)
#   ament_target_dependencies(car_link_node rclcpp std_msgs geometry_msgs uav_interfaces)
#   install(TARGETS car_link_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
