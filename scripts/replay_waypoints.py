#!/usr/bin/env python3
"""
航点回放脚本：按顺序依次导航到 YAML 中的航点。

用法:
  python3 replay_waypoints.py waypoints.yaml
"""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def load_waypoints(filepath: str) -> list:
    """简单解析 YAML，返回 [(x, y, qx, qy, qz, qw), ...]"""
    waypoints = []
    with open(filepath) as f:
        x = y = qx = qy = qz = qw = None
        for line in f:
            line = line.strip()
            if line.startswith("position:"):
                parts = line.split("[")[1].split("]")[0].split(",")
                x, y = float(parts[0]), float(parts[1])
            elif line.startswith("orientation:"):
                parts = line.split("[")[1].split("]")[0].split(",")
                qx, qy, qz, qw = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
                if None not in (x, y, qx, qy, qz, qw):
                    waypoints.append((x, y, qx, qy, qz, qw))
    return waypoints


class WaypointReplayer(Node):
    def __init__(self, waypoints: list, frame: str = "map"):
        super().__init__("waypoint_replayer")
        self.waypoints = waypoints
        self.frame = frame
        self.current = 0
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.get_logger().info(f"等待 navigate_to_pose action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("超时! 确认导航栈已启动")
            sys.exit(1)
        self.get_logger().info(f"已连接, 共 {len(waypoints)} 个航点")

    def send_next(self):
        if self.current >= len(self.waypoints):
            self.get_logger().info("全部完成!")
            rclpy.shutdown()
            return

        x, y, qx, qy, qz, qw = self.waypoints[self.current]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"→ 航点 {self.current + 1}/{len(self.waypoints)}: ({x:.3f}, {y:.3f})"
        )
        self._client.send_goal_async(goal).add_done_callback(self._goal_response)

    def _goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"航点 {self.current + 1} 被拒绝, 跳过")
            self.current += 1
            self.send_next()
            return

        self.get_logger().info(f"  执行中...")
        goal_handle.get_result_async().add_done_callback(self._result)

    def _result(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"  ✓ 到达航点 {self.current + 1}")
        else:
            self.get_logger().warn(f"  ✗ 航点 {self.current + 1} 失败 (status={status})")
        self.current += 1
        time.sleep(0.5)
        self.send_next()


def main():
    parser = argparse.ArgumentParser(description="回放录制好的航点")
    parser.add_argument("yaml_file", help="航点 YAML 文件")
    parser.add_argument("--frame", default="map", help="坐标系")
    args = parser.parse_args()

    waypoints = load_waypoints(args.yaml_file)
    if not waypoints:
        print(f"错误: {args.yaml_file} 中没有找到航点")
        sys.exit(1)

    print(f"加载 {len(waypoints)} 个航点:")
    for i, (x, y, _, _, qz, qw) in enumerate(waypoints, 1):
        yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * qz * qz)
        print(f"  {i}. ({x:.3f}, {y:.3f}) yaw={yaw:.2f} rad ({yaw*57.3:.0f}°)")

    rclpy.init()
    replayer = WaypointReplayer(waypoints, args.frame)
    replayer.send_next()
    rclpy.spin(replayer)


if __name__ == "__main__":
    main()
