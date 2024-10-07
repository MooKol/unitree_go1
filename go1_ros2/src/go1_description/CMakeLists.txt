cmake_minimum_required(VERSION 3.5)
project(go1_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Boost REQUIRED)


include_directories(
  ${Boost_INCLUDE_DIRS}  
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY xacro
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}
)

# Export package dependencies
ament_package()
