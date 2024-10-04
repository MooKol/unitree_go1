cmake_minimum_required(VERSION 3.5)
project(unitree_legged_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Declare message files
set(msg_files
  "msg/MotorCmd.msg"
  "msg/MotorState.msg"
  "msg/BmsCmd.msg"
  "msg/BmsState.msg"
  "msg/Cartesian.msg"
  "msg/IMU.msg"
  "msg/LED.msg"
  "msg/LowCmd.msg"
  "msg/LowState.msg"
  "msg/HighCmd.msg"
  "msg/HighState.msg"
)

# Generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Install directories
install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

install(PROGRAMS
    scripts/test_publisher.py
    scripts/test_subscriber.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
