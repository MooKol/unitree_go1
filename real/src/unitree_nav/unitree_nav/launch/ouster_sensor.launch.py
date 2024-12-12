import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from unitree_nav_launch_module import TernaryTextSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource




def generate_launch_description():
    viz_arg = DeclareLaunchArgument('viz', default_value='false',  description='Whether to use RViz for visualization')
            
    # Declare the sensor hostname parameter
    sensor_hostname = LaunchConfiguration('sensor_hostname', default='os-122139000982.local')
    lidar_mode = LaunchConfiguration('lidar_mode', default='512x20')
    #lidar_mode = LaunchConfiguration('lidar_mode', default='1024x20')
    sensor_qos_profile = LaunchConfiguration('sensor_qos_profile', default='reliable')
    use_system_default_qos = LaunchConfiguration('use_system_default_qos', default='true')
    timestamp_mode = LaunchConfiguration('timestamp_mode', default='TIME_FROM_ROS_TIME')
    viz = LaunchConfiguration('viz', default='false')
    #max_range = LaunchConfiguration('max_range', default='3.5')
    max_range = LaunchConfiguration('max_range', default='100.0')
    
    # Locate the original sensor.launch.xml
    ouster_ros_pkg = FindPackageShare(package='ouster_ros').find('ouster_ros')
    sensor_launch_path = os.path.join(ouster_ros_pkg, 'launch', 'sensor.launch.xml')

    # Include the sensor.launch.xml launch file
    include_sensor_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(sensor_launch_path),
        launch_arguments={'sensor_hostname': sensor_hostname, 'lidar_mode': lidar_mode, 'sensor_qos_profile': sensor_qos_profile, 'use_system_default_qos': use_system_default_qos, 'timestamp_mode': timestamp_mode,
            'viz': viz, 'max_range': max_range}.items(),
    )
    
    return LaunchDescription([
        viz_arg,
        include_sensor_launch,
    ])
