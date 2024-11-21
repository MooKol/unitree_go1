import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from unitree_nav_launch_module import TernaryTextSubstitution
from launch.conditions import IfCondition

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

    # Position and orientation
    # [X, Y, Z]
    position = [0, 0.0, 0.6]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
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
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

    map_odom_tf_publisher_node = Node(
        package='go1_navigation',
        executable='nav_tf_publisher',
        name='map_odom_transform_publisher',
        output='screen'
    )

    static_map_publisher_node = Node(
        package='go1_navigation',
        executable='static_map_publisher',
        name='static_map_publisher',
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
                'expected_update_rate':15.0,
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

    localize_only_arg = DeclareLaunchArgument(
        name='localize_only',
        default_value='false',
        choices=['true','false'],
        description='Localize only, do not change loaded map'
    )

    restart_map_arg = DeclareLaunchArgument(
                name='restart_map',
                default_value='false',
                choices=['true','false'],
                description='Delete previous map and restart'
            )

    use_rtabmapviz_arg = DeclareLaunchArgument(
                name='use_rtabmapviz',
                default_value='true',
                choices=['true','false'],
                description='Start rtabmapviz node'
            )

    rtabmap_log_level_arg = DeclareLaunchArgument(
                name='rtabmap_log_level',
                default_value='INFO',
                choices=['ERROR', 'WARN', 'INFO', 'DEBUG'],
                description='Set logger level for rtabmap.'
            )
    rtabmap_slam_node = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[{
            'frame_id':'laser_link',
            'subscribe_depth':False,
            'subscribe_rgb':False,
            'subscribe_scan_cloud':True,
            'approx_sync':True, # False
            'wait_for_transform': 0.3, #0.2,
            'use_sim_time':LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('scan_cloud', 'scan_pcd')
        ],
        arguments=[
            TernaryTextSubstitution(IfCondition(LaunchConfiguration('restart_map')), '-d', ''),
            'Mem/IncrementalMemory', TernaryTextSubstitution(IfCondition(LaunchConfiguration('localize_only')), 'false', 'true'),
            'Mem/InitWMWithAllNodes', LaunchConfiguration('localize_only'),
            'RGBD/ProximityMaxGraphDepth', '0',
            'RGBD/ProximityPathMaxNeighbors', '1',
            'RGBD/AngularUpdate', '0.05',
            'RGBD/LinearUpdate', '0.05',
            'RGBD/CreateOccupancyGrid', 'false',
            'Mem/NotLinkedNodesKept', 'false',
            'Mem/STMSize', '30',
            'Mem/LaserScanNormalK', '20',
            'Reg/Strategy', '1',
            'Icp/VoxelSize', '0.1',
            'Icp/PointToPlaneK', '20',
            'Icp/PointToPlaneRadius', '0',
            'Icp/PointToPlane', 'true',
            'Icp/Iterations', '10',
            'Icp/Epsilon', '0.001',
            'Icp/MaxTranslation', '3',
            'Icp/MaxCorrespondenceDistance', '1',
            'Icp/Strategy', '1',
            'Icp/OutlierRatio', '0.7',
            'Icp/CorrespondenceRatio', '0.2',
            '--ros-args',
            '--log-level',
            [
                TextSubstitution(text='rtabmap:='),
                LaunchConfiguration('rtabmap_log_level'),
            ],
        ]) 
    rtabmap_viz_node =  Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
                'frame_id':'laser_link',
                'odom_frame_id':'odom',
                'subscribe_odom_info':True,
                'subscribe_scan_cloud':True,
                'approx_sync':True, # False
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', 'scan_pcd')
            ],
            condition=IfCondition(LaunchConfiguration('use_rtabmapviz'))
        )


    """
    Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('unitree_nav'),
                    'config',
                    'nav.rviz'
                ])
            ],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        )
                IncludeLaunchDescription(
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
                        FindPackageShare('unitree_nav'),
                        'config',
                        'nav2_params.yaml'
                    ])
                ),
            ],
        )
    """

    # create and return launch description object
    return LaunchDescription(
        [
            localize_only_arg,
            restart_map_arg,
            use_rtabmapviz_arg, 
            rtabmap_log_level_arg,
            use_sim_time_arg,
            icp_odometry_log_level_arg,
            deskewing_arg,
            set_env_var_gazebo,
            world_file_name_arg,
            urdf_file_arg,
            start_world,
            spawn_robot,
            lidar_to_pcd_node,
            odom_from_lidar_node,
            launch_ros2_control,
            visualize_robot,
            rtabmap_slam_node,
            rtabmap_viz_node
        ]
    )
"""

    return LaunchDescription(
        [
            use_sim_time_arg,
            icp_odometry_log_level_arg,
            deskewing_arg,
            set_env_var_gazebo,
            world_file_name_arg,
            urdf_file_arg,
            start_world,
            spawn_robot,
            lidar_to_pcd_node,
            odom_from_lidar_node,
            launch_ros2_control,
            visualize_robot,
            map_odom_tf_publisher_node,
            static_map_publisher_node
        ]
    )
"""
