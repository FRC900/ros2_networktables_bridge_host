from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nt_port = LaunchConfiguration('nt_port')

    declare_nt_port = DeclareLaunchArgument(
        'nt_port', default_value='5800', description='NetworkTables port'
    )

    node = Node(
        package='ros_networktables_bridge_host',
        executable='ros_networktables_bridge_node.py',
        name='ros_networktables_bridge',
        output='screen',
        respawn=True,
        parameters=[
            {'is_server': True},
            {'port': nt_port},
        ],
    )

    return LaunchDescription([
        declare_nt_port,
        node,
    ])
