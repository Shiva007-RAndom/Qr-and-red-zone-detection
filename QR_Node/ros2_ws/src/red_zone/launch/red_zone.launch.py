"""Launch the red-zone detector.

Example:
    ros2 launch red_zone red_zone.launch.py image_topic:=/camera/image_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw',
                              description='Input image topic.'),
        DeclareLaunchArgument('use_compressed', default_value='false',
                              description='Subscribe to CompressedImage instead of Image.'),
        DeclareLaunchArgument('min_area', default_value='100',
                              description='Minimum patch area in pixels^2.'),
        DeclareLaunchArgument('process_every_n', default_value='1',
                              description='Process one in every N frames.'),
        DeclareLaunchArgument('publish_debug_image', default_value='true',
                              description='Publish an annotated image on ~/debug_image.'),
    ]

    node = Node(
        package='red_zone',
        executable='red_zone_node',
        name='red_zone_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'use_compressed': LaunchConfiguration('use_compressed'),
            'min_area': LaunchConfiguration('min_area'),
            'process_every_n': LaunchConfiguration('process_every_n'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
        }],
    )

    return LaunchDescription(args + [node])
