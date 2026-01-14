from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nt_host = LaunchConfiguration('nt_host')
    nt_port = LaunchConfiguration('nt_port')

    declare_nt_host = DeclareLaunchArgument(
        'nt_host', default_value='', description='NetworkTables host address'
    )

    declare_nt_port = DeclareLaunchArgument(
        'nt_port', default_value='1735', description='NetworkTables port'
    )

    node = Node(
        package='ros_networktables_bridge_host',
        executable='ros_networktables_bridge_node.py',
        name='ros_networktables_bridge',
        output='screen',
        respawn=True,
        parameters=[
            {'is_server': False},
            {'address': nt_host},
            {'port': nt_port},
        ],
    )

    return LaunchDescription([
        declare_nt_host,
        declare_nt_port,
        node,
    ])
