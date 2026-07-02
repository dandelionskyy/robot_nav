#!/usr/bin/env python3
"""
航点录制脚本：按回车记录当前 map->base_link 位姿，Ctrl+C 结束并保存为 YAML。

用法:
  ros2 run <pkg> record_waypoints.py
  或直接:
  python3 record_waypoints.py [--output waypoints.yaml] [--frame map] [--child base_link]
"""

import argparse
import os
import signal
import sys
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class WaypointRecorder(Node):
    def __init__(self, output_file: str, parent_frame: str, child_frame: str):
        super().__init__("waypoint_recorder")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.output_file = output_file
        self.waypoints = []
        self.count = 0

        self.get_logger().info(f"监听 TF: {parent_frame} -> {child_frame}")
        self.get_logger().info("按 Enter 记录航点, 输入 'd' 删除上一个, 'q' 保存退出")
        self.get_logger().info("-" * 50)

    def get_current_pose(self):
        """获取当前位姿，失败返回 None"""
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                self.parent_frame, self.child_frame, rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w

            # 从 quaternion 提取 yaw
            import math
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            return x, y, qx, qy, qz, qw, yaw
        except TransformException as e:
            self.get_logger().warn(f"获取 TF 失败: {e}")
            return None

    def record(self):
        pose = self.get_current_pose()
        if pose is None:
            return

        x, y, qx, qy, qz, qw, yaw = pose
        self.count += 1
        self.waypoints.append((x, y, qx, qy, qz, qw, yaw))

        self.get_logger().info(
            f"#{self.count} | x={x:.4f} y={y:.4f} yaw={yaw:.4f} rad ({yaw*57.3:.1f}°)"
        )

    def pop_last(self):
        if self.waypoints:
            self.count -= 1
            removed = self.waypoints.pop()
            self.get_logger().info(
                f"已删除 #{self.count+1} (x={removed[0]:.4f} y={removed[1]:.4f})"
            )
        else:
            self.get_logger().info("没有可删除的航点")

    def save(self):
        if not self.waypoints:
            self.get_logger().info("没有航点可保存")
            return

        # 备份旧文件
        if os.path.exists(self.output_file):
            bak = self.output_file + ".bak"
            os.rename(self.output_file, bak)
            self.get_logger().info(f"已备份旧文件 -> {bak}")

        with open(self.output_file, "w") as f:
            f.write(f"# 航点录制: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# 坐标系: {self.parent_frame}\n")
            f.write(f"# 共 {len(self.waypoints)} 个航点\n")
            f.write(f"# 格式: x, y, qx, qy, qz, qw, yaw\n\n")
            f.write("waypoints:\n")
            for i, wp in enumerate(self.waypoints, 1):
                x, y, qx, qy, qz, qw, yaw = wp
                f.write(f"  - id: {i}\n")
                f.write(f"    position: [{x:.4f}, {y:.4f}, 0.0]\n")
                f.write(f"    orientation: [{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}]\n")
                f.write(f"    yaw: {yaw:.4f}  # {yaw*57.3:.1f}°\n")

        self.get_logger().info(f"已保存 {len(self.waypoints)} 个航点到: {self.output_file}")


def main():
    parser = argparse.ArgumentParser(description="录制机器人航点")
    parser.add_argument("--output", default=None, help="输出 YAML 文件路径")
    parser.add_argument("--frame", default="map", help="父坐标系")
    parser.add_argument("--child", default="base_link", help="子坐标系")
    args = parser.parse_args()

    rclpy.init()

    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = os.path.join(
            os.getcwd(), f"waypoints_{timestamp}.yaml"
        )

    recorder = WaypointRecorder(args.output, args.frame, args.child)

    try:
        while rclpy.ok():
            rclpy.spin_once(recorder, timeout_sec=0.1)

            # 非阻塞读取
            import select
            if select.select([sys.stdin], [], [], 0.1)[0]:
                line = sys.stdin.readline().strip().lower()
                if line == "q":
                    recorder.save()
                    break
                elif line == "d":
                    recorder.pop_last()
                elif line == "":
                    recorder.record()
                else:
                    recorder.get_logger().info("回车=记录, d=删除, q=保存退出")

    except KeyboardInterrupt:
        recorder.get_logger().info("\n收到中断信号，正在保存...")
        recorder.save()
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
