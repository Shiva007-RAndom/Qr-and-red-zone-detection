"""Attach the qr_match node to the SkyScan Gazebo world.

Bridges the x500's downward camera from Gazebo into ROS 2 and starts
qr_match_node against it. The reference is the correct target (qr_c), the
one the start QR says to deliver to.

    ros2 launch launch/skyscan_qr.launch.py
    ros2 launch launch/skyscan_qr.launch.py reference:=qr_a   # test a wrong target
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# PX4 appends an instance suffix (_0) to the spawned model name.
GZ_CAM_TOPIC = ('/world/aerothon_mission2/model/x500_mono_cam_down_0'
                '/link/camera_link/sensor/camera/image')


def generate_launch_description():
    gz_topic = LaunchConfiguration('gz_camera_topic')
    return LaunchDescription([
        DeclareLaunchArgument('gz_camera_topic', default_value=GZ_CAM_TOPIC),
        DeclareLaunchArgument('reference_qr_path', default_value='',
                              description='Optional reference image; empty = capture from camera.'),
        DeclareLaunchArgument('match_threshold', default_value='0.85'),
        DeclareLaunchArgument('model_size', default_value='s'),
        DeclareLaunchArgument('process_every_n', default_value='3'),

        # Gazebo -> ROS image bridge, remapped to /camera/image_raw
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='camera_bridge', output='screen',
            arguments=[[gz_topic, '@sensor_msgs/msg/Image[gz.msgs.Image']],
            remappings=[(GZ_CAM_TOPIC, '/camera/image_raw'),
                        (gz_topic, '/camera/image_raw')],
        ),

        Node(
            package='red_zone', executable='red_zone_node',
            name='red_zone_node', output='screen', emulate_tty=True,
            parameters=[{
                'image_topic': '/camera/image_raw',
                'process_every_n': 2,
            }],
        ),

        Node(
            package='qr_match', executable='qr_match_node',
            name='qr_match_node', output='screen', emulate_tty=True,
            parameters=[{
                'reference_qr_path': LaunchConfiguration('reference_qr_path'),
                'image_topic': '/camera/image_raw',
                'match_threshold': LaunchConfiguration('match_threshold'),
                'model_size': LaunchConfiguration('model_size'),
                'process_every_n': LaunchConfiguration('process_every_n'),
            }],
        ),
    ])
