cmake_minimum_required(VERSION 3.8)
project(uav_bringup VERSION 2.0.0)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch systemd udev
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/install_boot_service.sh
    scripts/prepare_uav_clock.sh
    scripts/start_uav_system.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
