import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart

def generate_launch_description():
    ld = LaunchDescription()
    pkg_path = FindPackageShare('41068_ignition_bringup')

    # -------------------
    # Arguments
    # -------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    ))

    rviz_flag = LaunchConfiguration('rviz')
    nav2_flag = LaunchConfiguration('nav2')
    ld.add_action(DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='true', description='Launch Nav2'))

    # -------------------
    # 1. Forest world
    # -------------------
    ignition_pkg = get_package_share_directory('41068_ignition_bringup')
    forest_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ignition_pkg, 'launch', 'quadruped_forest.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world', default='simple_trees')
        }.items()
    )
    ld.add_action(forest_world)

    # -------------------
    # 2. Quadruped setup
    # -------------------
    quadruped_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ignition_pkg, 'launch', 'quadruped_setup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    # Delay robot spawn to let world load
    delayed_robot_spawn = TimerAction(
        period=25.0,  # seconds
        actions=[quadruped_spawn]
    )
    ld.add_action(delayed_robot_spawn)

    # -------------------
    # 3. Nav2 + SLAM (delayed start to wait for setup)
    # -------------------
    navigation = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_path, 'launch', 'quadruped_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(nav2_flag)
    )

    # Delay Nav2/SLAM start by 15s to give robot setup time
    nav2_delayed = TimerAction(
        period=40.0,
        actions=[navigation]
    )
    ld.add_action(nav2_delayed)

    # -------------------
    # 4. RViz
    # -------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_path, 'config', 'quadruped_GUI.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_flag)
    )
    # Delay RViz a bit more (Nav2 gets 10s, so give RViz 12s)
    rviz_delayed = TimerAction(
        period=52.0,  # Nav2 gets 10s, RViz starts a bit later
        actions=[rviz]
    )
    ld.add_action(rviz_delayed)
     # -------------------
    # 6. Visual Icons Publisher
    # -------------------
    visual_icons_node = Node(
        package='visual_icons',
        executable='icon_publisher',
        name='icon_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_flag)
    )
    # Delay RViz a bit more (Nav2 gets 10s, so give RViz 12s)
    visual_icons_node_delayed = TimerAction(
        period=55.0,  # Nav2 gets 10s, RViz starts a bit later
        actions=[visual_icons_node]
    )
    ld.add_action(visual_icons_node_delayed)
    # -------------------
    # 7. Behavior Controller (always on)
    # -------------------
    behavior_controller = Node(
        package='robot_behavior_controller',
        executable='behavior_controller_node',
        name='behavior_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Optionally delay a bit to allow world and robot to spawn
    behavior_controller_delayed = TimerAction(
        period=55.0,
        actions=[behavior_controller]
    )
    ld.add_action(behavior_controller_delayed)

    return ld
