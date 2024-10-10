import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable, AppendEnvironmentVariable
from launch import LaunchContext

import xacro

def launch_setup(context, *args, **kwargs):
    rname_val = LaunchConfiguration('rname').perform(context)
    wname_val = LaunchConfiguration('wname').perform(context)

    robot_path = FindPackageShare(rname_val + '_description')
    #print(robot_path.perform(context))

    world_path = PathJoinSubstitution([FindPackageShare('unitree_gazebo'), 'worlds', wname_val + '.world'])
    #print(world_path.perform(context))

    robot_description_path = os.path.join(
        get_package_share_directory('go1_description'),
        'xacro',
        'robot.xacro'
    )
    robot_description = xacro.process_file(
        robot_description_path, mappings={'DEBUG': 'false'}
    ).toxml()


    """

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gzserver_cmd = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
                        ),
                launch_arguments={'gz_args': ['-r -s -v4 ', world_path], 'on_exit_shutdown': 'true'}.items())

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )
    """

    # Load URDF into parameter server
    robot_description_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description},
                {'publish_frequency': 1000.0},
                {'use_sim_time': True},
            ],
        )
    gazebo_model_path = os.path.join(get_package_prefix("go1_description"), "share")

    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # Launch Gazebo server (gzserver) with Gazebo Classic
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Launch Gazebo client (gzclient) for visualization
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        )
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', 'go1_robot',
            '-topic', 'robot_description'
        ],
        output='both'
        )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'go1_robot',
            '-topic', '/robot_description'
        ],
        output='screen'
    )


    #return [robot_description_node, gzserver_cmd, gzclient_cmd, SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("go1_description"), "share")), spawn_robot_node]
    return [robot_description_node, gzclient_cmd, gzserver_cmd, spawn_robot_node]

def generate_launch_description():
    wname_arg = DeclareLaunchArgument('wname', default_value='earth')
    rname_arg = DeclareLaunchArgument('rname', default_value='go1')

    return LaunchDescription([
        wname_arg, rname_arg,
        OpaqueFunction(function = launch_setup)
        ])
