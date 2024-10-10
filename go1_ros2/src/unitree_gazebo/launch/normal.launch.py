import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch import LaunchContext

def launch_setup(context, *args, **kwargs):
    rname_val = LaunchConfiguration('rname').perform(context)
    wname_val = LaunchConfiguration('wname').perform(context)

    robot_path = FindPackageShare(rname_val + '_description')
    #print(robot_path.perform(context))

    world_path = PathJoinSubstitution([FindPackageShare('unitree_gazebo'), 'worlds', wname_val + '.world'])
    #print(world_path.perform(context))

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gzserver_cmd = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
                        ),
                launch_arguments={'gz_args': ['-r -s -v4 ', world_path], 'on_exit_shutdown': 'true'}.items())


    return [gzserver_cmd]

def generate_launch_description():
    wname_arg = DeclareLaunchArgument('wname', default_value='earth')
    rname_arg = DeclareLaunchArgument('rname', default_value='go1')

    return LaunchDescription([
        wname_arg, rname_arg,
        OpaqueFunction(function = launch_setup)
        ])
