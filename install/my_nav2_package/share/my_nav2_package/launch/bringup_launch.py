from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Stage 시뮬레이터 실행
    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('stage_ros'),
                'launch',
                'stage.launch.py'
            )
        )
    )

    # Nav2 브링업 실행
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': os.path.join(
                get_package_share_directory('my_nav2_package'),
                'maps',
                'my_map.yaml'
            ),
            'params_file': os.path.join(
                get_package_share_directory('my_nav2_package'),
                'params',
                'nav2_params.yaml'
            ),
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        stage_launch,
        nav2_launch
    ])
