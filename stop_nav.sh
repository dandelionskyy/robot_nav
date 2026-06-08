#!/bin/bash
# 停止机器人导航系统
SESSION="robot_nav"
tmux kill-session -t "$SESSION" 2>/dev/null && echo "导航系统已停止" || echo "没有运行中的导航 session"
