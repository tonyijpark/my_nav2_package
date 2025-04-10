import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.actions import TimerAction


def generate_launch_description():
    bringup_dir = get_package_share_directory('my_nav2_package')
    launch_dir = os.path.join(bringup_dir, 'launch')
    map_yaml_file = os.path.join(bringup_dir, 'maps', 'simple_map.yaml')
    params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')

    print("map_yaml_file: {}", map_yaml_file)

    return LaunchDescription([
        # 로봇 상태 퍼블리셔 노드 실행
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     parameters=[{'use_sim_time': True}],
        #     output='screen',
        # ),

        # 맵 서버 노드 실행
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                {'use_sim_time': True, 'yaml_filename': map_yaml_file}
            ],
            output='screen',
        ),

        # AMCL 노드 실행
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[params_file, {'use_sim_time': True}],
            output='screen',
        ),

        # 컨트롤러 서버 노드 실행
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[params_file, {'use_sim_time': True}],
            output='screen',
        ),

        # 플래너 서버 노드 실행
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[params_file, {'use_sim_time': True}],
            output='screen',
        ),

        # 행동 관리자 노드 실행
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[params_file, {'use_sim_time': True}],
            output='screen',
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        # Lifecycle Manager (map_server 전용)
        # TimerAction(
        #     period=2.0,  # 2초 지연 후 실행 (map_server 초기화 대기)
        #     actions=[
        #         Node(
        #             package='nav2_lifecycle_manager',
        #             executable='lifecycle_manager',
        #             name='lifecycle_manager_map',
        #             output='screen',
        #             parameters=[{
        #                 'use_sim_time': True,
        #                 'autostart': True,
        #                 'node_names': ['map_server']
        #             }]
        #         )
        #     ]
        # ),

        #브링업 노드 실행
        # Node(
        #     package='nav2_bringup',
        #     executable='navigation_launch.py',
        #     name='nav2_bringup',
        #     parameters=[params_file],
        #     output='screen',
        # )
    ])
        

