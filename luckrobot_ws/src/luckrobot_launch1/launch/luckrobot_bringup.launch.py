import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ==========================================
    # 阶段 1：传感器 (事件级错峰启动)
    # ==========================================

    livox_driver_event = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="============ [事件 1] T+4s：拉起 Livox 雷达驱动 ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360s_launch.py'])
                )
            )
        ]
    )

    # ==========================================
    # 阶段 2：高阶算法与宏观控制错峰拉起
    # ==========================================

    # T+10s: 启动 FAST-LIO + 全局定位
    open3d_event = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="============ [事件 2] T+10s：拉起 FAST-LIO + Open3D 全局定位 ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('open3d_loc'), 'launch', 'localization_3d_g1.launch.py'])
                )
            )
        ]
    )

    # T+30s: 启动 Nav2 导航栈
    nav2_event = TimerAction(
        period=30.0,
        actions=[
            LogInfo(msg="============ [事件 3] T+30s：平稳拉起 Nav2 导航栈 ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('nav2_luckrobot'), 'launch', 'nav2.launch.py'])
                )
            )
        ]
    )

    # T+45s: 启动航点导航调度节点
    nav_manager_event = TimerAction(
        period=45.0,
        actions=[
            LogInfo(msg="============ [事件 4] T+45s：Nav2 就绪！拉起中央导航调度节点 ============"),
            launch_ros.actions.Node(
                package="wheel_controller",
                executable="nav_manager_node",
                name="nav_manager_node",
                output="screen",
                parameters=[{"use_sim_time": False}]
            )
        ]
    )

    # ==========================================
    # 组装返回
    # ==========================================
    return LaunchDescription([
        LogInfo(msg="============ [系统启动] 纯定时无阻塞流水线事件引擎已启动 ============"),
        livox_driver_event,
        open3d_event,
        nav2_event,
        nav_manager_event,
    ])
