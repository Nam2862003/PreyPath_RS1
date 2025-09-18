import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Declare sim time
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true'
    ))

    # Package paths
    ignition_pkg = get_package_share_directory('41068_ignition_bringup')
    gazebo_pkg = get_package_share_directory('gazebo_sim')

    # 1. Forest world (master RViz + Nav2)
    forest_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ignition_pkg, 'launch', 'forest_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world', default='simple_trees'),
            'rviz': 'True',
            'nav2': 'True'
        }.items()
    )

    # 2. Quadruped spawn only (no RViz/Nav2)
    quadruped_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo_multi_nav2_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'with_rviz': 'False',
            'with_nav2': 'False',
            'namespace': 'robot1'
        }.items()
    )

    ld.add_action(forest_world)
    ld.add_action(quadruped_spawn)

    # 3. EKF localization for quadruped
    ekf_config = os.path.join(gazebo_pkg, 'config', 'ekf.yaml')
    quad_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='robot1',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Add all actions
    ld.add_action(forest_world)
    ld.add_action(quadruped_spawn)
    ld.add_action(quad_ekf)
    return ld
