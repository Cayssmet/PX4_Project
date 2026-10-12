cmake_minimum_required(VERSION 3.8)
project(cartographer_config VERSION 1.0.0)

find_package(ament_cmake REQUIRED)

install(DIRECTORY configuration_files/
  DESTINATION share/${PROJECT_NAME}/configuration_files)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
