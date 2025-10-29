from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Pythonサブサーバ (decode)
        Node(
            package='path_reuse_method',
            executable='decode_server.py',
            name='decode_path_seed_sub_server',
            output='screen',
        ),
        # Pythonサブサーバ (encode)
        Node(
            package='path_reuse_method',
            executable='encode_server.py',
            name='encode_path_seed_sub_server',
            output='screen',
        ),
        # C++のメインサーバ
        Node(
            package='path_reuse_method',
            executable='path_seed_server',
            name='path_seed_server',
            output='screen',
        ),
    ])
