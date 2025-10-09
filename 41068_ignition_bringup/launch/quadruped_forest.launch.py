from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
# from launch.conditions import IfCondition
# from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = FindPackageShare('41068_ignition_bringup')

    # which world to load
    world = LaunchConfiguration('world')
    ld.add_action(DeclareLaunchArgument(
        'world', default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo','forest_arena', 'world_gen','big_world']
    ))

    # launch Ignition with chosen forest world
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path,
                                               'worlds',
                                               [world, '.sdf']]),
                         ' -r']
        }.items()
    )
    ld.add_action(gazebo)

    return ld
