from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mode = LaunchConfiguration('mode', default='slam2d')

    sensors_launch = os.path.join(
        get_package_share_directory('preypath_sensors'),
        'launch', 'sensors_minimal.launch.py'
    )
    slam2d_launch = os.path.join(
        get_package_share_directory('preypath_slam'),
        'launch', 'slam2d_stub.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='slam2d'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sensors_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(slam2d_launch)),
    ])

