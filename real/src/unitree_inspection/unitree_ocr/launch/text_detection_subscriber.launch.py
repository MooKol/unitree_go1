from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_local_camera',
            default_value='false',
            choices=['true', 'false'],
            description='Open local USB camera'
        ),

        DeclareLaunchArgument(
            name='swap_rb',
            default_value='false',
            choices=['true', 'false'],
            description='Swap the red and blue color channels in the input image',
        ),
        DeclareLaunchArgument(
            name='display_size_multiplier',
            default_value='1.0',
            description='Change the size of the displayed image',
        ),
        Node(
            package='unitree_ocr',
            executable='text_detection_subscriber',
            output='screen',
            parameters=[{
                'detection.model_path':
                    PathJoinSubstitution([
                        FindPackageShare('unitree_ocr'),
                        'models',
                        'frozen_east_text_detection.pb',
                    ]),
                'recognition.model_path':
                    PathJoinSubstitution([
                        FindPackageShare('unitree_ocr'),
                        'models',
                        'crnn_cs.onnx',
                    ]),
                'recognition.vocabulary_path':
                    PathJoinSubstitution([
                        FindPackageShare('unitree_ocr'),
                        'models',
                        'alphabet_94.txt',
                    ]),
                'swap_rb': LaunchConfiguration('swap_rb'),
                'display_size_multiplier': LaunchConfiguration('display_size_multiplier'),
            }],
            remappings=[
                ('/image/compressed', '/camera/camera/color/image_raw/compressed'),
                ('/camera_info', '/camera/camera/color/camera_info'),
            ],
        )
    ])
