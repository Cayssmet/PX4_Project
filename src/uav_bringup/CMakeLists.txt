cmake_minimum_required(VERSION 3.8)
project(uav_bringup VERSION 1.4.1)

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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_full_system_launch_config
    test/test_full_system_launch_config.py
  )
endif()

ament_package()
