"""Launch the QR match node.

Example:
    ros2 launch qr_match qr_match.launch.py \
        reference_qr_path:=/home/user/dummy_qr.png \
        image_topic:=/camera/color/image_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'reference_qr_path', default_value='',
            description='Absolute path to the reference QR image. Leave empty '
                        'to capture the reference from the camera via '
                        '~/capture_reference.'),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/image_raw',
            description='Input image topic.'),
        DeclareLaunchArgument(
            'use_compressed', default_value='false',
            description='Subscribe to CompressedImage instead of Image.'),
        DeclareLaunchArgument(
            'match_threshold', default_value='0.85',
            description='Score at or above which a QR counts as a match.'),
        DeclareLaunchArgument(
            'model_size', default_value='s',
            description='qrdet YOLO model size: n, s, m or l.'),
        DeclareLaunchArgument(
            'process_every_n', default_value='1',
            description='Process one in every N frames.'),
        DeclareLaunchArgument(
            'log_every_frame', default_value='false',
            description='Log every processed frame, not just matches.'),
    ]

    node = Node(
        package='qr_match',
        executable='qr_match_node',
        name='qr_match_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'reference_qr_path': LaunchConfiguration('reference_qr_path'),
            'image_topic': LaunchConfiguration('image_topic'),
            'use_compressed': LaunchConfiguration('use_compressed'),
            'match_threshold': LaunchConfiguration('match_threshold'),
            'model_size': LaunchConfiguration('model_size'),
            'process_every_n': LaunchConfiguration('process_every_n'),
            'log_every_frame': LaunchConfiguration('log_every_frame'),
        }],
    )

    return LaunchDescription(args + [node])
