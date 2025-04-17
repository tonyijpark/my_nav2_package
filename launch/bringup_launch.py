import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Path setup
    pkg_share = get_package_share_directory('my_nav2_package')
    map_yaml_path = os.path.join(pkg_share, 'maps', 'simple_map.yaml')
    params_file = os.path.join(pkg_share, 'params', 'nav2_params.yaml')

    # Declare Launch Arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=map_yaml_path, description='Full path to map yaml file')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=params_file, description='Full path to the parameters file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')

    # Launch Configuration Variables
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Static transform publisher (map -> odom)
    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time, 'always_send_full_map': True}]
    )

    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/scan', '/base_scan')]
    )

    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen'
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']}]
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    map_republisher = Node(
            package='my_nav2_package',
            executable='map_republisher.py',
            name='map_republisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )

    return LaunchDescription([
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        static_tf_pub,
        map_server,
        amcl,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
        map_republisher,
        rviz
    ])
