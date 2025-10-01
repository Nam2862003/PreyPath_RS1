import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction
)
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
import xacro
from launch_ros.substitutions import FindPackageShare

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

    # -------------------
    # Paths
    # -------------------

   # Base remappings for robot nodes
    # remappings = [
    #     ("/tf", "tf"),
    #     ("/tf_static", "tf_static"),
    #     ("/scan", "scan"),
    #     ("/odom", "odometry/filtered")
    # ]
    # remaps_tf = [('tf', '/tf'), ('tf_static', '/tf_static'), ("/scan", "scan")]
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
        # remappings=remaps_tf
    )

    # Publish odom -> base_link transform **using robot_localization**
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
        #  remappings=remaps_tf
    )
    # -------------------
    # Spawn in Gazebo
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
    # Gazebo bridges
    # ------------------
    # # Clock bridge
    # bridge_params = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')
    # ros_gz_bridge_clock = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         '--ros-args',
    #         '-p',
    #         f'config_file:={bridge_params}',
    #     ]
    # )

    # ros_gz_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     namespace=namespace,
    #     name='ros_gz_bridge',
    #     output='screen',
    #     arguments=[
    #         f'/{namespace}/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU',
    #         f'/{namespace}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
    #         f'/{namespace}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
    #         f'/{namespace}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
    #         f'/{namespace}/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
    #         f'/{namespace}/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
    #         f'/{namespace}/color/image_rect@sensor_msgs/msg/Image@gz.msgs.Image',
    #         # f'/{namespace}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
    #     ]
    # )

    # start_gazebo_ros_image_bridge_cmd = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     namespace=namespace,
    #     arguments=['color/image_raw', 'color/image_rect'],
    #     output='screen',
    # )
    bridge_params = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "--ros-args", "-p", f"config_file:={bridge_params}"
        ]
    )

    # -------------------
    # Controllers
    # -------------------
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        name='joint_state_broadcaster',
        arguments=['joint_state_broadcaster'],
        output='screen',
        # remappings=remaps_tf
    )
    joint_group_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        name='joint_group_controller',
        arguments=['joint_group_controller'],
        output='screen',
        # remappings=remaps_tf
    )
    controller = Node(
        package='quadropted_controller',
        executable='robot_controller_gazebo.py',
        name='quadruped_controller',
        namespace=namespace,
        output='screen',
        # remappings=remaps_tf
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
                'imu_topic': f'/{namespace}/imu_plugin/out',
                'base_frame_id': "base_link",
                'odom_frame_id': "odom",
                'clock_topic': f'/clock',
                'enable_odom_tf': False,
            }],
        # remappings=remaps_tf
    )

    cmd_vel_pub = Node(
        package='quadropted_controller',
        executable='cmd_vel_pub.py',
        namespace=namespace,
        name='cmd_vel_pub',
        output='screen',
        # remappings=remaps_tf
    #      remappings=[
    #     ('cmd_vel', '/cmd_vel_nav'),   # <-- key line
    # ]
    )
    relay_nav2_cmd = Node(
    package="topic_tools",
    executable="relay",
    arguments=["/cmd_vel_nav", f"/{namespace}/cmd_vel"],   # Nav2 → robot1/cmd_vel
    output="screen"
    )

    relay_teleop_cmd = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/cmd_vel", f"/{namespace}/cmd_vel"],       # Teleop → robot1/cmd_vel
        output="screen"
    )
    # # Fake battery state
    # fake_bms = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'topic', 'pub', f'/{namespace}/battery_state',
    #         'sensor_msgs/msg/BatteryState',
    #         "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, voltage: 24.0, percentage: 0.8, capacity: 10.0}",
    #         '-r', '1'
    #     ],
    #     output='log'
    # )

    # -------------------
    # Localization EKF
    # -------------------
 
    # -------------------
    # Group all robot processes
    # -------------------
    robot_group = GroupAction([
        node_robot_state_publisher,
        spawn_entity,
        # ros_gz_bridge_clock,
        ros_gz_bridge,
        # start_gazebo_ros_image_bridge_cmd,
        joint_state_broadcaster,
        joint_group_controller,
        controller,
        cmd_vel_pub,
        odom,
        robot_localization_node,
        relay_nav2_cmd,
        relay_teleop_cmd,
        # fake_bms,
    ])

    ld.add_action(robot_group)

    return ld