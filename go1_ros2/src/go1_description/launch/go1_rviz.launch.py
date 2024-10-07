import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    robot_description_path = os.path.join(
        get_package_share_directory('go1_description'),
        'xacro',
        'robot.xacro'
    )

    robot_description = xacro.process_file(
        robot_description_path, mappings={'DEBUG': 'false'}
    ).toxml()

    return LaunchDescription([
        # Joint State Publisher Node (with GUI)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            parameters=[{'use_gui': True}],
        ),

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description},
                {'publish_frequency': 1000.0}
            ],
        ),

        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('go1_description'), 'launch', 'check_joint.rviz')],
        ),
    ])
