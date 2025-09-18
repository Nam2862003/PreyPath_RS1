from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = FindPackageShare('41068_ignition_bringup')

    # sim time arg
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    ))

    # which world to load
    world = LaunchConfiguration('world')
    ld.add_action(DeclareLaunchArgument(
        'world', default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    ))

    # flags for rviz/nav2
    rviz = LaunchConfiguration('rviz')
    nav2 = LaunchConfiguration('nav2')
    ld.add_action(DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='false', description='Launch Nav2'))

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

    # navigation stack
    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(nav2)
    )
    ld.add_action(navigation)

    # rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_path, 'config', '41068.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )
    ld.add_action(rviz_node)

    return ld
