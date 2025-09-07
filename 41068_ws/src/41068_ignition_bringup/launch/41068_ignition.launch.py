import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # -----------------------
    # Get paths
    # -----------------------
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path, 'config'])

    # -----------------------
    # Launch arguments
    # -----------------------
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='False',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    world = LaunchConfiguration('world')
    ld.add_action(world_launch_arg)

    # -----------------------
    # Husky description
    # -----------------------
    husky_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_path, 'urdf', 'Husky_URDF', 'husky.urdf.xacro'])
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': husky_description,
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(robot_state_publisher_node)

    # (Manual control only -> EKF removed)

    # Spawn Husky in Gazebo
    husky_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '0.4']
    )
    ld.add_action(husky_spawner)

    # -----------------------
    # Quadruped description
    # -----------------------
    quadruped_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_path, 'urdf', 'Quadruped_URDF', 'robot.xacro']),
            ' controller_config:=',
            PathJoinSubstitution([config_path, 'robot_controller.yaml'])
        ]),
        value_type=str
    )

    quadruped_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='quadruped',
        parameters=[{'robot_description': quadruped_description,
                     'use_sim_time': use_sim_time}]
    )
    ld.add_action(quadruped_state_publisher)

    quadruped_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/quadruped/robot_description',
                   '-entity', 'quadruped',
                   '-z', '0.4',
                   '-x', '0.0', '-y', '-1.0']
    )
    ld.add_action(quadruped_spawner)

    # -----------------------
    # Quadruped Controllers (load after spawn)
    # -----------------------
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'joint_state_broadcaster',
            '--controller-manager', '/quadruped/controller_manager'
        ],
        output='screen'
    )

    load_forward_position_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'forward_position_controller',
            '--controller-manager', '/quadruped/controller_manager'
        ],
        output='screen'
    )

    # Ensure sequential startup
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=quadruped_spawner,
            on_exit=[load_joint_state_broadcaster],
        )
    ))
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_forward_position_controller],
        )
    ))

    # -----------------------
    # Ignition Gazebo
    # -----------------------
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

    # -----------------------
    # Gazebo bridge (kept as-is)
    # -----------------------
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # -----------------------
    # Robot control GUI (manual_control)
    # -----------------------
    robot_control_launch = os.path.join(
        get_package_share_directory('robot_control'), 'launch', 'robot_control.launch.py'
    )

    manual_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_control_launch)
    )
    ld.add_action(manual_control)

    # -----------------------
    # RViz (optional)
    # -----------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path, '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # (Manual control only -> Nav2 include removed)

    return ld
