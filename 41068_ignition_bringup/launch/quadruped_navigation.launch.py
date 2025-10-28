from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    config_path = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'config'])

    # Additional command line arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Flag to enable use_sim_time'
    )

    # Start Simultaneous Localisation and Mapping (SLaM)
    slam = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                             'launch', 'online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_path, 'quadruped_slam_params.yaml'])
        }.items()
    )
    # Start Navigation Stack
    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_path, 'quadruped_nav2_params.yaml'])
        }.items()
    )

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(slam)
    ld.add_action(navigation)
    # # Delay Nav2 after SLAM (so /map & TF are ready)
    # nav2_delayed = TimerAction(period=.0, actions=[navigation])
    # ld.add_action(nav2_delayed)

    return ld
