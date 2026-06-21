#!/bin/bash
# 一键启动机器人导航系统 (tmux 分窗口)
# 用法: ./start_nav.sh         → 默认航点模式
#       ./start_nav.sh manual  → 手动导航模式 (仅Nav2, 不启动航点调度)

set -e

SESSION="robot_nav"
MODE="${1:-auto}"

# 工作空间路径 (按实际修改)
LIVOX_WS="$HOME/robot_nav/livox_ws"
MID360_WS="$HOME/robot_nav/mid360s_ws"
LOC_WS="$HOME/robot_nav/fastlio_localization"
LUCK_WS="$HOME/robot_nav/luckrobot_ws"

# 如果 session 已存在，先杀掉
tmux kill-session -t "$SESSION" 2>/dev/null || true

# 创建 session，第一个窗口作为信息窗
tmux new-session -d -s "$SESSION" -n "info"
tmux send-keys -t "$SESSION:info" "echo '===== 机器人导航系统 ====='"
tmux send-keys -t "$SESSION:info" Enter
tmux send-keys -t "$SESSION:info" "echo '窗口1: livox (雷达驱动)'"
tmux send-keys -t "$SESSION:info" Enter
tmux send-keys -t "$SESSION:info" "echo '窗口2: fastlio (里程计)'"
tmux send-keys -t "$SESSION:info" Enter
tmux send-keys -t "$SESSION:info" "echo '窗口3: localization (全局定位)'"
tmux send-keys -t "$SESSION:info" Enter
tmux send-keys -t "$SESSION:info" "echo '窗口4: nav (导航/航点)'"
tmux send-keys -t "$SESSION:info" Enter
tmux send-keys -t "$SESSION:info" "echo ''"
tmux send-keys -t "$SESSION:info" "echo '按 Ctrl+B 再按数字键切换窗口'"
tmux send-keys -t "$SESSION:info" Enter

# ---- 窗口1: Livox 雷达驱动 ----
tmux new-window -t "$SESSION" -n "livox"
tmux send-keys -t "$SESSION:livox" "echo '>>> 启动 Livox 雷达驱动 <<<'"
tmux send-keys -t "$SESSION:livox" Enter
tmux send-keys -t "$SESSION:livox" "source $LIVOX_WS/install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360s_launch.py"
tmux send-keys -t "$SESSION:livox" Enter

# ---- 窗口2: FAST-LIO (T+4s) ----
tmux new-window -t "$SESSION" -n "fastlio"
tmux send-keys -t "$SESSION:fastlio" "echo '>>> 启动 FAST-LIO (等4秒) <<<'"
tmux send-keys -t "$SESSION:fastlio" Enter
tmux send-keys -t "$SESSION:fastlio" "sleep 4"
tmux send-keys -t "$SESSION:fastlio" Enter
tmux send-keys -t "$SESSION:fastlio" "source $MID360_WS/install/setup.bash && ros2 launch fast_lio_map mapping.launch.py rviz:=false"
tmux send-keys -t "$SESSION:fastlio" Enter

# ---- 窗口3: 全局定位 (T+8s) ----
tmux new-window -t "$SESSION" -n "localization"
tmux send-keys -t "$SESSION:localization" "echo '>>> 启动全局定位 (等8秒) <<<'"
tmux send-keys -t "$SESSION:localization" Enter
tmux send-keys -t "$SESSION:localization" "sleep 8"
tmux send-keys -t "$SESSION:localization" Enter
tmux send-keys -t "$SESSION:localization" "source $LOC_WS/install/setup.bash && ros2 launch open3d_loc open3d_loc_g1.launch.py use_rviz:=false"
tmux send-keys -t "$SESSION:localization" Enter

# ---- 窗口4: 导航 (T+15s) ----
tmux new-window -t "$SESSION" -n "nav"
tmux send-keys -t "$SESSION:nav" "echo '>>> 启动导航 (等15秒) <<<'"
tmux send-keys -t "$SESSION:nav" Enter
tmux send-keys -t "$SESSION:nav" "sleep 15"
tmux send-keys -t "$SESSION:nav" Enter
if [ "$MODE" = "manual" ]; then
    tmux send-keys -t "$SESSION:nav" "source $LUCK_WS/install/setup.bash && ros2 launch nav2_luckrobot nav2.launch.py"
else
    tmux send-keys -t "$SESSION:nav" "source $LUCK_WS/install/setup.bash && ros2 launch nav2_luckrobot nav2.launch.py && ros2 run wheel_controller nav_manager_node"
fi
tmux send-keys -t "$SESSION:nav" Enter

# 切回 info 窗口并 attach
tmux select-window -t "$SESSION:info"
tmux attach-session -t "$SESSION"
