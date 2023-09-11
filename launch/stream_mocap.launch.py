from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    return LaunchDescription([
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='stream_mocap',
            name='stream_mocap',
        ),

        Node(
            package='qualisys_mocap',
            executable='qualisys_node',
            parameters=[
                {'server': '192.168.123.2'},
            ],
        ),
    ])
