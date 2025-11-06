import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    LogInfo
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import xacro


def generate_launch_description():
    ld = LaunchDescription()

    # -------------------
    # Launch arguments
    # -------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    ld.add_action(declare_use_sim_time)

    # -------------------
    # Robot basic info 
    # -------------------
    namespace = "robot1"
    x_pose, y_pose, z_pose = 0.0, 0.0, 0.8   # starting pose

    package_name = '41068_ignition_bringup'
    pkg_path = get_package_share_directory(package_name)

    xacro_file = os.path.join(
        get_package_share_directory('go2_description'),
        'xacro',
        'robot.xacro'
    )

    # -------------------
    # Robot description
    # -------------------
    robot_desc = xacro.process_file(
        xacro_file,
        mappings={'robot_name': "robot1"}
    ).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=namespace,
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }],
    )

    robot_localization_file_path = os.path.join(
        pkg_path,
        'config',
        'quadruped_localiztion.yaml'
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=namespace,
        output='screen',
        parameters=[robot_localization_file_path,
                    {'use_sim_time': use_sim_time}],
    )

    # -------------------
    # Spawn in Gazebo (short-lived)
    # -------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=[
            '-topic', f'/{namespace}/robot_description',
            '-name', f'{namespace}',
            '-allow_renaming', 'true',
            '-x', str(x_pose),
            '-y', str(y_pose),
            '-z', str(z_pose),
        ],
        output='screen'
    )

    # -------------------
    # Nodes that should start AFTER spawn_entity
    # -------------------
    bridge_params = os.path.join(pkg_path, 'config', 'quadruped_gazebo_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        name="ros_gz_bridge",
        output="screen",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"]
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        name='joint_state_broadcaster',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    joint_group_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        name='joint_group_controller',
        arguments=['joint_group_controller'],
        output='screen',
    )
    controller = Node(
        package='quadropted_controller',
        executable='robot_controller_gazebo.py',
        name='quadruped_controller',
        namespace=namespace,
        output='screen',
    )
    odom = Node(
        package='quadropted_controller',
        executable='QuadrupedOdometryNode.py',
        name='odom',
        namespace=namespace,
        output='screen',
        parameters=[{
            "verbose": False,
            'publish_rate': 50,
            'open_loop': False,
            'has_imu_heading': True,
            'is_gazebo': True,
            'imu_topic': f'/{namespace}/imu',
            'base_frame_id': "base_link",
            'odom_frame_id': "odom",
            'clock_topic': f'/clock',
            'enable_odom_tf': False,
        }],
    )
    # camera_tf_pub = Node(
    # package='tf2_ros',
    # executable='static_transform_publisher',
    # name='camera_optical_tf',
    # namespace=namespace,
    # arguments=[
    #     '0', '0', '0', '-1.5708', '0', '-1.5708',
    #     'camera_link', 'camera_optical_frame'
    # ],
    # output='screen'
    # )

    cmd_vel_pub = Node(
        package='quadropted_controller',
        executable='cmd_vel_pub.py',
        namespace=namespace,
        name='cmd_vel_pub',
        output='screen',
    )
    relay_nav2_cmd = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/cmd_vel_nav", f"/{namespace}/cmd_vel"],
        output="screen"
    )
    relay_teleop_cmd = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/cmd_vel", f"/{namespace}/cmd_vel"],
        output="screen"
    )

    # -------------------
    # Launch order
    # -------------------
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity)  # spawn first

    # everything else waits for spawn_entity to finish
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    LogInfo(msg="Robot spawned, starting bridges, controllers and localization..."),
                    ros_gz_bridge,
                    joint_state_broadcaster,
                    joint_group_controller,
                    controller,
                    cmd_vel_pub,
                    odom,
                    robot_localization_node,
                    # camera_tf_pub,
                    relay_nav2_cmd,
                    relay_teleop_cmd
                ]
            )
        )
    )

    return ld
