import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_description_path = os.path.join(
        get_package_share_directory('robot_description'))
    
    robot_control_path = os.path.join(
        get_package_share_directory('robot_control'))
    
    ignition_pkg_path = FindPackageShare('41068_ignition_bringup')

    # -----------------------
    # Set gazebo sim resource path
    # -----------------------
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=str(Path(robot_description_path).parent.resolve())
    )

    # -----------------------
    # World argument (bringup style)
    # -----------------------
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo']
    )
    world = LaunchConfiguration('world')

    # -----------------------
    # Ignition Gazebo
    # -----------------------
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([ignition_pkg_path,
                                               'worlds',
                                               [world, '.sdf']]),
                         ' -r']
        }.items()
    )

    # -----------------------
    # Process robot xacro
    # -----------------------
    xacro_file = os.path.join(robot_description_path,
                              'robot',
                              'robot.xacro')

    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # -----------------------
    # Spawn robot
    # -----------------------
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'acs_robot',
                   '-allow_renaming', 'false'],
    )

    # -----------------------
    # Controllers
    # -----------------------
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen'
    )

    # -----------------------
    # Bridge
    # -----------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/acs_robot/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen'
    )

    # -----------------------
    # Robot control
    # -----------------------
    manual_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_control_path, 'launch', 'robot_control.launch.py'),
        )
    )

    # -----------------------
    # Return LaunchDescription
    # -----------------------
    return LaunchDescription([
        world_launch_arg,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_forward_position_controller],
            )
        ),
        gazebo_resource_path,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        manual_control,
        bridge,
    ])
