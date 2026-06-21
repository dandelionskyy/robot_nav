import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # 获取目录
    nav2_luckrobot_dir = get_package_share_directory('nav2_luckrobot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    rviz_config_dir = os.path.join(nav2_luckrobot_dir, 'rviz', 'my_nav2_view.rviz')
    
    # 获取 Launch 配置参数
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    use_rviz = launch.substitutions.LaunchConfiguration('use_rviz', default='false')

    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(nav2_luckrobot_dir, 'maps', 'test_map.yaml'))

    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(nav2_luckrobot_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value='false'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path),
        launch.actions.DeclareLaunchArgument('use_rviz', default_value='false',
                                             description='Whether to start RViz2'),

        # ---- Map Server ----
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_path, 'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        # ---- 核心导航层 (禁用自带定位，定位由外部 fastlio_localization 提供) ----
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'use_localization': 'false'}.items(),   # ← 关键：禁用 AMCL
        ),

        # ---- RViz ----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(use_rviz)
        ),
    ])