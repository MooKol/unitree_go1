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

# this is the function launch  system will look for


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        choices=['true','false'],
        description='Use simulation (Gazebo) clock if true'
    )

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
                'expected_update_rate': 15.0,
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
            'frame_id':'lidar',
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
            'Icp/VoxelSize', '0.05',
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
            'Grid/CellSize', '0.05',
            'Grid/RangeMax', '10',
            'Grid/GroundIsObstacle', 'false',
            'Grid/RayTracing', 'true',
            'Grid/3D', 'true',
            'Grid/FootprintLength', '0.82',
            'Grid/FootprintWidth', '0.7',
            'Grid/FootprintHeight', '0.4',
            'Grid/NormalsSegmentation', 'false',
            'Grid/NormalK', '40',
            'Grid/MaxGroundAngle', '30',
            'Grid/MaxGroundHeight', '0.2',
            'Grid/ClusterRadius', '0.2',
            'Grid/MinClusterSize', '30',
            'Grid/FlatObstacleDetected', 'false',
            'Grid/MaxObstacleHeight', '0.0',
            'Grid/NoiseFilteringRadius', '0.0',
            'Grid/NoiseFilteringMinNeighbors', '0.0',
            'GridGlobal/Eroded', 'true',
            'GridGlobal/FootprintRadius', '0.255',
            'GridGlobal/OccupancyThr', '0.5',
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
                'frame_id':'lidar',
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

    # create and return launch description object
    return LaunchDescription(
        [  
            icp_odometry_log_level_arg,
            deskewing_arg,
            use_sim_time_arg,
            use_rtabmapviz_arg,
            rtabmap_log_level_arg,
            localize_only_arg,
            restart_map_arg,
            rtabmap_slam_node,
            rtabmap_viz_node, 
        ]
    )

