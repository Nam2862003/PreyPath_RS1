import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    pkg_path = FindPackageShare('41068_ignition_bringup')

    # sim time arg
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    ))
       # flags for rviz/nav2
    rviz = LaunchConfiguration('rviz')
    nav2 = LaunchConfiguration('nav2')
    ld.add_action(DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='false', description='Launch Nav2'))
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
            'world': LaunchConfiguration('world', default='simple_trees')
        }.items()
    )
    ld.add_action(forest_world)
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
    ld.add_action(quadruped_spawn)

    # # 3. EKF localization for quadruped
    # ekf_config = os.path.join(gazebo_pkg, 'config', 'ekf.yaml')
    # quad_ekf = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     namespace='robot1',
    #     output='screen',
    #     parameters=[ekf_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )
    # ld.add_action(quad_ekf)
    #4 . Rviz and Nav2 with SLAM Toolbox
    # flags for rviz/nav2
    # rviz = LaunchConfiguration('rviz')
    # nav2 = LaunchConfiguration('nav2')
    # ld.add_action(DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz'))
    # ld.add_action(DeclareLaunchArgument('nav2', default_value='false', description='Launch Nav2'))

    # navigation stack
    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(nav2)
    )
    ld.add_action(navigation)

    # # rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_path, 'config', '41068.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )
    ld.add_action(rviz_node)
    return ld
