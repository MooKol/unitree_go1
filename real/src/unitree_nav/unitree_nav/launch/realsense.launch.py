from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Path to the realsense2_camera package's launch file
    rs_launch_path = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"),
        "launch",
        "rs_launch.py"
    ])
    # 1280x720x30, 640x480x30, 1920x1080x30
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_path),
            launch_arguments={
                "enable_depth": "false",
                "pointcloud.enable": "true",
                "rgb_camera.color_profile": "1920x1080x30",
            }.items(),
        )
    ])

