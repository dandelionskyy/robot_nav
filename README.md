# 家庭服务机器人 — 导航系统

基于 ROS 2 Humble + Mid-360 激光雷达的 3D 建图定位 + 2D 导航。

## 项目来源

| 上游项目 | 用途 | 本仓库对应目录 |
|---------|------|-------------|
| [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) + [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | Mid-360 雷达驱动 | `livox_ws/` |
| [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) | 里程计/建图 | `mid360s_ws/` |
| [FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID) | Open3D 全局定位 | `fastlio_localization/` |
| Nav2 自定义规划器/控制器 + 底盘调度 | 自研 | `luckrobot_ws/` |

## 相比上游的改动

### FAST-LIO (`mid360s_ws`)

- **雷达话题改为直连 Livox 驱动**：`lid_topic: /livox/lidar`，`imu_topic: /livox/imu`
- **限制视场角**：`blind=0.8`, `fov_degree=270`，过滤机身遮挡
- **launch 默认关闭 rviz**：`rviz:=false`
- **支持雷达倒置**：倒装时将 `extrinsic_R` 改为绕X轴180°: `[1,0,0, 0,-1,0, 0,0,-1]`

### 全局定位 (`fastlio_localization`)

- **定位节点内部转发 `/Odometry_loc`**：订阅 FAST-LIO 的 `/Odometry`，原样转发为 `/Odometry_loc` 供 Nav2 做速度源
- **发布 TF `map→odom`**：ICP 配准修正矩阵通过 TF 广播，不再单独发 `/motionlink2map`
- **重映射话题**：`/map→/map_3d`，`/scan→/scan_3d`，避免与 Nav2 冲突
- **Open3D 点云采样优化**：降低体素分辨率、限制参与配准的点数，适配 Jetson Nano 性能
- **pointcloud_to_laserscan**：将 3D 点云压成 `/scan_2d`，范围 ±135°，高度 -0.4~0.2m，过滤机身
- **TF 树重构**：`map → odom → base_link`，去掉了 motion_link 中间层

### Nav2 导航 (`luckrobot_ws`)

- **自定义直线全局规划器** (`nav2_custom_planner`)：忽略障碍物，start→goal 直线几何路径采样，用于任务穿越模式
- **自定义纯追踪控制器** (`nav2_custom_controller`)：原地旋转/直行双模式，双层角度阈值
- **`nav_manager_node`**：订阅 `/cmd_mode` (UInt8) 触发对应航点，到达后发布 `/cmd_vel_mode`
- Nav2 参数适配：`robot_radius=0.28`，`odom_topic=/Odometry_loc`，代价地图膨胀参数等

## 部署

### 1. 系统依赖

```bash
sudo apt install ros-humble-nav2-bringup ros-humble-pointcloud-to-laserscan
```

Livox SDK2 需要按[官方文档](https://github.com/Livox-SDK/Livox-SDK2)编译安装。

### 2. 环境变量 (`~/.bashrc`)

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/robot_nav/fastdds_shm.xml
```

### 3. 编译

```bash
cd ~/robot_nav
cd livox_ws && colcon build --symlink-install && cd ..
cd mid360s_ws && colcon build --symlink-install && cd ..
cd fastlio_localization && colcon build --symlink-install && cd ..
cd luckrobot_ws && colcon build --symlink-install && cd ..
```

### 4. 必改配置

| 文件 | 参数 | 说明 |
|------|------|------|
| `open3d_loc_g1.launch.py` | `map_file` | 指向你的 `.pcd` 地图文件 |
| `mid360.yaml` | `map_file_path` | 同上 |
| `mid360.yaml` | `extrinsic_T` | IMU-LiDAR 外参 |
| `mid360.yaml` | `extrinsic_R` | 雷达正装用单位阵，**倒装绕X轴180°**: `[1,0,0, 0,-1,0, 0,0,-1]` |
| `nav_manager_node.cpp` | `target_locations_` | 航点坐标表 |

## 启动

### 一键启动

```bash
./start_nav.sh         # 完整模式：雷达→里程计→定位→Nav2+航点
./start_nav.sh manual  # 手动模式：仅启动到 Nav2，Rviz 点目标
./stop_nav.sh          # 停止
```

### 分步启动 (调试)

```bash
# 1. 雷达
source livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py

# 2. 里程计 (正装雷达)
source mid360s_ws/install/setup.bash
ros2 launch fast_lio_map mapping.launch.py rviz:=false
# 雷达倒装时用: config_file:=mid360_inverted.yaml
# 建图完成后保存: ros2 service call /map_save std_srvs/srv/Trigger

# 3. 定位 (等待打印 "localizatiaon initialize success!!!!")
source fastlio_localization/install/setup.bash
ros2 launch open3d_loc open3d_loc_g1.launch.py use_rviz:=false

# 4. Nav2
source luckrobot_ws/install/setup.bash
ros2 launch nav2_luckrobot nav2.launch.py

# 5. (可选) 航点调度
ros2 run wheel_controller nav_manager_node
ros2 topic pub /cmd_mode std_msgs/msg/UInt8 "data: 2"
```

## 数据流

```
Livox ─→ FAST-LIO ─→ Localization ─→ Nav2
 点云      /Odometry    TF: map→odom    导航/控制
          TF: odom→     /Odometry_loc
              base_link  /scan_2d
```

TF 树：`map → odom → base_link`

| TF 边 | 发布者 |
|-------|--------|
| `odom→base_link` | FAST-LIO (里程计, 10Hz) |
| `map→odom` | 定位节点 (ICP 修正, 2Hz) |

## 调试

```bash
ros2 topic hz /Odometry           # FAST-LIO 是否正常
ros2 topic hz /Odometry_loc       # 定位转发是否正常
ros2 topic hz /scan_2d            # 2D 扫描是否有输出
ros2 run tf2_tools view_frames    # TF 树是否完整 (map→odom→base_link)
```

动一下就飘 = `map→odom` TF 没发布 → 用 Rviz 给初始位姿，或检查 `.pcd` 地图路径是否正确。

## 航点录制

录制固定路线航点，用于回放或 nav_manager_node 编排。

```bash
# 确保导航已启动，机器人就位
python3 scripts/record_waypoints.py --output ~/waypoints.yaml
```

| 按键 | 功能 |
|------|------|
| **回车** | 记录当前位置 (map→base_link) |
| **d + 回车** | 删除上一个航点 |
| **q + 回车** | 保存并退出 |
| **Ctrl+C** | 保存并退出 |

输出为 YAML 格式，每个航点包含 `position`、`orientation` (四元数) 和 `yaw`。

### PGM 地图角度校准

如果 PGM 地图与真实环境存在固定角度偏差（走一段后偏差越来越大），调整 pcd2pgm 的 yaw 参数：

1. 编辑 `/home/dandelion/Point cloud processing/pcd2pgm/src/config/pcd2pgm.yaml`
2. 修改 `odom_to_lidar_odom[5]` (Z 轴旋转角，单位弧度)
   - 增大 → PGM 逆时针旋转
   - 减小 → PGM 顺时针旋转
3. 重新运行 `pcd2pgm` 生成新 PGM 并替换到 `nav2_luckrobot/maps/`
