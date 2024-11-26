import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource

# this is the function launch  system will look for


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        choices=['true','false'],
        description='Use simulation (Gazebo) clock if true'
    )

    world_file_name = LaunchConfiguration('world_file_name')
    urdf_file = LaunchConfiguration('urdf_file')

    world_file_name_arg = DeclareLaunchArgument(
        'world_file_name',
        default_value='test_latest.world'
    )

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='robot.xacro'
    )

    position_x = LaunchConfiguration('position_x')
    position_y = LaunchConfiguration('position_y')
    position_z = LaunchConfiguration('position_z')
    
    orientation_x = LaunchConfiguration('orientation_x')
    orientation_y = LaunchConfiguration('orientation_y')
    orientation_z = LaunchConfiguration('orientation_z')
    
    position_x_arg = DeclareLaunchArgument(
        'position_x',
        default_value='0.0'
    )
    
    position_y_arg = DeclareLaunchArgument(
        'position_y',
        default_value='0.0'
    )
    
    position_z_arg = DeclareLaunchArgument(
        'position_z',
        default_value='0.0'
    )
    
    orientation_x_arg = DeclareLaunchArgument(
        'orientation_x',
        default_value='0.0'
    )
    
    orientation_y_arg = DeclareLaunchArgument(
        'orientation_y',
        default_value='0.0'
    )
    
    orientation_z_arg = DeclareLaunchArgument(
        'orientation_z',
        default_value='0.0'
    )

    # Base Name or robot
    robot_name = "GO1"
    
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go1_gazebo'), 'launch'),
            '/start_world.launch.py']),
    launch_arguments={'world_file_name': world_file_name}.items(),
    )
    
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   robot_name,
                   '-x', position_x, '-y', position_y, '-z', position_z,
                   '-R', orientation_x, '-P', orientation_y, '-Y', orientation_z,
                   '-topic', '/robot_description'
                   ]
    )

    map_odom_tf_publisher_node = Node(
        package='go1_navigation',
        executable='nav_tf_publisher',
        name='map_odom_transform_publisher',
        output='screen'
    )

    map_file_path = os.path.join(get_package_share_directory('go1_navigation'), 'map', 'map.yaml')

    map_publisher_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}],
        remappings=[('map', 'map')]
    )
    
    manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    launch_ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go1_gazebo'), 'launch'),
            '/controllers_go1.launch.py'])
    )

    visualize_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go1_description'), 'launch', 'go1_visualize.launch.py')
            ]),
    launch_arguments={'use_joint_state_publisher': 'False',
                      'use_sim_time': "True",
                      'urdf_file': urdf_file}.items(),
    )

    lidar_to_pcd_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[('cloud_in', '/scan_pcd'), ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_link',
            'min_height': -0.2,
            'max_height': 0.3,
            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.01,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 90.0
        }]
    )
    set_env_var_gazebo = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("go1_description"), "share"))
           
           
           
    icp_odometry_log_level_arg =  DeclareLaunchArgument(
            name='icp_odometry_log_level',
            default_value='INFO',
            choices=['ERROR', 'WARN', 'INFO', 'DEBUG'],
            description='Set logger level for icp_odometry. Can set to WARN to reduce message output from this node.'
        )
    deskewing_arg =  DeclareLaunchArgument(
            name='deskewing',
            default_value='false',
            choices=['true','false'],
            description='Enable lidar deskewing'
        )

    odom_from_lidar_node = Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
                'frame_id':'base_link',
                'odom_frame_id':'odom',
                'wait_for_transform':0.3, # 0.2
                'expected_update_rate':10.0,
                'deskewing': LaunchConfiguration('deskewing'),
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', '/scan_pcd')
            ],
            arguments=[
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/VoxelSize', '0.1',
                'Icp/Epsilon', '0.001',
                'Icp/PointToPlaneK', '20',
                'Icp/PointToPlaneRadius', '0',
                'Icp/MaxTranslation', '2',
                'Icp/MaxCorrespondenceDistance', '1',
                'Icp/Strategy', '1',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.01',
                'Odom/ScanKeyFrameThr', '0.6',
                'OdomF2M/ScanSubtractRadius', '0.1',
                'OdomF2M/ScanMaxSize', '15000',
                'OdomF2M/BundleAdjustment', 'false',
                '--ros-args',
                '--log-level',
                [
                    TextSubstitution(text='icp_odometry:='),
                    LaunchConfiguration('icp_odometry_log_level')
                ],
            ])

    control_delayed = TimerAction(
            period=30.0,
            actions=[launch_ros2_control]
            )

    # create and return launch description object
    return LaunchDescription(
        [
            position_x_arg,
            position_y_arg,
            position_z_arg,
            orientation_x_arg,
            orientation_y_arg,
            orientation_z_arg,
            use_sim_time_arg,
            icp_odometry_log_level_arg,
            deskewing_arg,
            set_env_var_gazebo,
            world_file_name_arg,
            urdf_file_arg,
            start_world,
            spawn_robot,
            launch_ros2_control,
            visualize_robot,
        ]
    )

