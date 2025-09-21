import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

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
    # Package paths
    # -------------------
    package_name = 'gazebo_sim'
    pkg_path = get_package_share_directory(package_name)
    robots_file_path = os.path.join(pkg_path, 'config', 'robots.yaml')

    # Load robot config
    with open(robots_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    robots = yaml_data['robots']
    # -------------------
    # Remappings
    # -------------------
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/scan", "scan"),
        ("/odom", "odometry/filtered")
    ]

    # Clock bridge (ros_ign_bridge)
    bridge_params = os.path.join(pkg_path, 'config', 'gazebo_bridge.yaml')
    ros_ign_bridge_clock = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': bridge_params,
                     'use_sim_time': use_sim_time}]
    )
    ld.add_action(ros_ign_bridge_clock)

    # -------------------
    # Robot loop
    # -------------------
    last_action = None
    for robot in robots:
        namespace = robot['name']
        robot_name = robot['name']

        # Build robot description from xacro
        xacro_file = os.path.join(
            get_package_share_directory('go2_description'),
            'xacro', 'robot.xacro'
        )
        robot_desc = xacro.process_file(
            xacro_file, mappings={'robot_name': robot_name}
        ).toxml()
        params_robot_state_publisher = {
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }

        # State publisher
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[params_robot_state_publisher],
            remappings=remappings
        )

        # Spawn in Ignition Gazebo
        spawn_entity = Node(
            package='ros_ign_gazebo',
            executable='create',
            namespace=namespace,
            arguments=[
                '-topic', f'/{namespace}/robot_description',
                '-name', f'{namespace}_my_bot',
                '-allow_renaming', 'true',
                '-x', str(robot['x_pose']),
                '-y', str(robot['y_pose']),
                '-z', str(robot['z_pose']),
            ],
            output='screen'
        )

        # Bridges
        ros_ign_bridge = Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            name='ros_ign_bridge',
            output='screen',
            arguments=[
                f'/{namespace}/imu_plugin/out@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                f'/{namespace}/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                f'/{namespace}/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                f'/{namespace}/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
                f'/{namespace}/color/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                f'/{namespace}/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                f'/{namespace}/color/image_rect@sensor_msgs/msg/Image@ignition.msgs.Image',
                f'/{namespace}/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
            ]
        )
        # Controllers
        joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            name='joint_state_broadcaster',
            arguments=['joint_state_broadcaster'],
            output='screen',
            remappings=remappings
        )

        joint_group_controller = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            name='joint_group_controller',
            arguments=['joint_group_controller'],
            output='screen',
            remappings=remappings
        )
        controller = Node(
            package='quadropted_controller',
            executable='robot_controller_gazebo.py',
            name='quadruped_controller',
            namespace=namespace,
            output='screen',
            remappings=remappings
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
                'clock_topic': '/clock',
                'enable_odom_tf': True,
            }],
            remappings=remappings
        )
        # Localization
        robot_localization_file_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
        start_robot_localization_cmd = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=namespace,
            output='screen',
            parameters=[robot_localization_file_path,
                        {'use_sim_time': use_sim_time}],
            remappings=remappings
        )
        # Group robot processes
        robot_control = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            joint_state_broadcaster,
            joint_group_controller,
            controller,
            odom,
            start_robot_localization_cmd,
        ])
        robot_group = GroupAction([
            node_robot_state_publisher,
            spawn_entity,
            ros_ign_bridge,
            robot_control
        ])
        if last_action is None:
            ld.add_action(robot_group)
        else:
            spawn_robot_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[robot_group]
                )
            )
            ld.add_action(spawn_robot_event)
        last_action = joint_group_controller
    return ld
