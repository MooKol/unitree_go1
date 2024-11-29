cmake_minimum_required(VERSION 3.5)
project(go1_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
