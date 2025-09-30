from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='preypath_sensors',
            executable='heartbeat',
            name='sensors_heartbeat',
            output='screen'
        ),
    ])

