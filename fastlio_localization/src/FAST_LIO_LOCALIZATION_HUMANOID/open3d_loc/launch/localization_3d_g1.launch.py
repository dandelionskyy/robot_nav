from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # 声明 use_rviz 参数
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz2'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    # 获取包路径
    open3d_loc_share = FindPackageShare('open3d_loc')

    # 注意: FAST-LIO 由 mid360s_ws 单独启动，此处不再包含

    # 包含 open3d_loc 的 launch 文件
    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                open3d_loc_share,
                'launch',
                'open3d_loc_g1.launch.py'
            ])
        ])
    )

    # RViz 节点配置
    rviz_config_path = PathJoinSubstitution([
        open3d_loc_share,
        'rviz_cfg',
        'fastlio.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_map_cur',
        arguments=['-d', rviz_config_path],
        output='screen',
        prefix='nice',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        use_rviz_arg,
        open3d_loc_launch,
        rviz_node
    ])
