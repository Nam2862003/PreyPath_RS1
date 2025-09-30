from launch import LaunchDescription
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='[preypath_slam] slam2d_stub.launch.py loaded (replace with slam_toolbox soon)'),
    ])

