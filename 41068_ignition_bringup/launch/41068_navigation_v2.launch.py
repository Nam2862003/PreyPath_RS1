from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    config_path = PathJoinSubstitution(
        [FindPackageShare("41068_ignition_bringup"), "config"]
    )

    # Additional command line arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_launch_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Flag to enable use_sim_time",
    )

    # Start Simultaneous Localisation and Mapping (SLAM)
        # Start Simultaneous Localisation and Mapping (SLAM)
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        namespace='robot1', 
        parameters=[
            {"use_sim_time": use_sim_time,
            # "odom_frame": "odom",
            # "map_frame": "map",
            # "base_frame": "base_link",
            # "scan_topic": "/robot1/scan",
            # "odom_topic": "/odometry/filtered"
            },
            PathJoinSubstitution([config_path, "slam_params.yaml"])
        ],
        # remappings=[
        #     ("/scan", "/robot1/scan"),
        #     ("/odom", "/odometry/filtered")
        # ]
    )


    # Nav2 core nodes (explicit, not via nav2_bringup)
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[PathJoinSubstitution([config_path, "nav2_params.yaml"])],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[PathJoinSubstitution([config_path, "nav2_params.yaml"])],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[PathJoinSubstitution([config_path, "nav2_params.yaml"])],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[PathJoinSubstitution([config_path, "nav2_params.yaml"])],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[PathJoinSubstitution([config_path, "nav2_params.yaml"])],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[PathJoinSubstitution([config_path, "nav2_params.yaml"])],
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[PathJoinSubstitution([config_path, "nav2_params.yaml"])],
    )
    remappings_initial = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/scan", "scan"),
        # ("/odom", "odometry/filtered")
    ]
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "smoother_server",
                    "bt_navigator",
                    "behavior_server",
                    "waypoint_follower",
                    "velocity_smoother",
                ],
            }
        ],
        remappings=remappings_initial
    )
    map_saver_server = Node(
    package="nav2_map_server",
    executable="map_saver_server",
    name="map_saver_server",
    output="screen",
    parameters=[PathJoinSubstitution([config_path, "nav2_params.yaml"])]
    )

    # Add actions
    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(slam)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(smoother_server)
    ld.add_action(bt_navigator)
    ld.add_action(behavior_server)
    ld.add_action(waypoint_follower)
    ld.add_action(velocity_smoother)
    ld.add_action(lifecycle_manager)
    ld.add_action(map_saver_server)
    return ld
