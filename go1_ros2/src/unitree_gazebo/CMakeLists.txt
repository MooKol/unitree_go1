cmake_minimum_required(VERSION 3.5)  # ROS 2 requires CMake 3.5 or higher
project(unitree_gazebo)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)  # for Gazebo-specific headers and libraries
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_legged_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Include directories
include_directories(
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
    ${ament_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME}
)

#add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
#target_link_libraries(unitreeDrawForcePlugin ${GAZEBO_LIBRARIES} rclcpp::rclcpp)
#ament_target_dependencies(unitreeDrawForcePlugin
#    rclcpp
#    geometry_msgs
#)

ament_package()
