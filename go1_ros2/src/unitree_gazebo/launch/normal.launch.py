import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit

import xacro

def launch_setup(context, *args, **kwargs):
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")

    wname = LaunchConfiguration('wname').perform(context)

    robot_description_path = os.path.join(get_package_share_directory('go1_description'), 'xacro', 'robot.xacro')
    world_path = PathJoinSubstitution([FindPackageShare('unitree_gazebo'), 'worlds', wname + '.world'])
    links_config = os.path.join(get_package_share_directory('go1_config'), "config/links/links.yaml")

    robot_description = xacro.process_file(robot_description_path, mappings={'DEBUG': 'false'}).toxml()
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

    set_env_var_gazebo = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("go1_description"), "share"))
    gzserver_cmd = LaunchDescription([
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s',
            'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
    ])

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            'go1_robot',
            '-topic',
            '/robot_description',
            "-robot_namespace",
            "",
            "-x",
            world_init_x,
            "-y",
            world_init_y,
            "-z",
            world_init_z,
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            world_init_heading,
        ],
        output='screen'
    )

    joint_states_controller_node = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_states_controller']
            )

    joint_group_effort_controller_node = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_group_effort_controller']
            )


    contact_sensor = Node(
        package="unitree_gazebo",
        executable="contact_sensor",
        output="screen",
        parameters=[{"use_sim_time": True}, links_config],
    )

    return [robot_description_node, set_env_var_gazebo,
            gzclient_cmd, gzserver_cmd, spawn_robot_cmd,
            RegisterEventHandler(
                OnProcessExit(
                    target_action = spawn_robot_cmd,
                    on_exit = [
                        TimerAction(
                            period=10.0,
                            actions=[
                                joint_states_controller_node, joint_group_effort_controller_node

                            ]
                        ),

                    ]
                )
            )
                        ]

def generate_launch_description():
    declare_wname = DeclareLaunchArgument('wname', default_value='earth')
    declare_rname = DeclareLaunchArgument('rname', default_value='go1')

    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="1.0")
    declare_world_init_heading = DeclareLaunchArgument("world_init_heading", default_value="0.0")

    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=os.path.join(get_package_share_directory('go1_description'), "config/ros_control.yaml"),
    )

    return LaunchDescription([
        declare_wname,
        declare_rname,
        declare_world_init_x,
        declare_world_init_y,
        declare_world_init_z,
        declare_world_init_heading,
        declare_ros_control_file,
        OpaqueFunction(function = launch_setup)
        ])
