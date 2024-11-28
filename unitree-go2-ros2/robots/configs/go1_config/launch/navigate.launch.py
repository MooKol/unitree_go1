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




def generate_launch_description():

    nav2_bringup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ),
                launch_arguments=[
                    ('params_file',
                        PathJoinSubstitution([
                            FindPackageShare('go1_description'),
                            'config',
                            'nav2_params.yaml'
                        ])
                    ),
                ],
            )
            
    rviz_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'rviz_launch.py'
                    ])
                ))
            
    # create and return launch description object
    return LaunchDescription(
        [  
            nav2_bringup,
            rviz_node
        ]
    )

