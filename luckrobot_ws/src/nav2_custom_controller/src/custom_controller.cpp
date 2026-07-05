#include "nav2_custom_controller/custom_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <cmath>

namespace nav2_custom_controller {

void CustomController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.1));
  node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(0.7));
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.8));
  node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
}

void CustomController::cleanup() { RCLCPP_INFO(node_->get_logger(), "清理控制器"); }
void CustomController::activate() {}
void CustomController::deactivate() {}

geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) {

  if (global_plan_.poses.empty()) throw nav2_core::PlannerException("路径为空");

  geometry_msgs::msg::PoseStamped pose_in_globalframe;
  if (!nav2_util::transformPoseInTargetFrame(pose, pose_in_globalframe, *tf_,
        global_plan_.header.frame_id, 0.1)) {
    throw nav2_core::PlannerException("TF转换失败");
  }

  // 裁剪已走过的路径点（从 getNearestTargetPose 中提取出来）
  if (global_plan_.poses.size() > 1) {
    int nearest_idx = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < global_plan_.poses.size(); i++) {
      double dist = std::hypot(
          global_plan_.poses[i].pose.position.x - pose_in_globalframe.pose.position.x,
          global_plan_.poses[i].pose.position.y - pose_in_globalframe.pose.position.y);
      if (dist < min_dist) { min_dist = dist; nearest_idx = i; }
    }
    if (nearest_idx > 0) {
      global_plan_.poses.erase(global_plan_.poses.begin(),
                               global_plan_.poses.begin() + nearest_idx);
    }
  }

  auto target_pose = getNearestTargetPose(pose_in_globalframe);
  auto final_goal = global_plan_.poses.back();

  // 距终点距离
  double dist_to_final = std::hypot(
      final_goal.pose.position.x - pose_in_globalframe.pose.position.x,
      final_goal.pose.position.y - pose_in_globalframe.pose.position.y);

  // 机器人朝向
  double robot_yaw = tf2::getYaw(pose_in_globalframe.pose.orientation);
  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);

  // 前瞻点变换到 body 帧
  double dx = target_pose.pose.position.x - pose_in_globalframe.pose.position.x;
  double dy = target_pose.pose.position.y - pose_in_globalframe.pose.position.y;
  double lx =  dx * cos_yaw + dy * sin_yaw;   // 前向分量
  double ly = -dx * sin_yaw + dy * cos_yaw;   // 横向分量（左正）

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = node_->get_clock()->now();
  cmd_vel.twist.linear.y = 0.0;  // 差速机器人无横向移动

  // 终点领域迟滞：进入 0.30m，退出 0.40m（避免边界反复切换）
  constexpr double FINAL_ENTER_DIST = 0.30;
  constexpr double FINAL_EXIT_DIST  = 0.40;
  constexpr double DECEL_DIST       = 1.00;   // 减速区长度
  constexpr double YAW_DEADBAND     = 0.05;   // yaw 死区 rad，抑制小幅度振荡
  constexpr double MIN_VX_RATIO     = 0.02;   // 最小速度系数，避免太慢卡住

  if (dist_to_final <= FINAL_ENTER_DIST) {
    in_final_approach_ = true;
  } else if (dist_to_final > FINAL_EXIT_DIST) {
    in_final_approach_ = false;
  }

  // 全程渐变减速：DECEL_DIST 外全速，进入减速区后线性降速
  double vx_ratio = (dist_to_final >= DECEL_DIST) ? 1.0
                     : std::max(MIN_VX_RATIO, dist_to_final / DECEL_DIST);

  if (in_final_approach_) {
    // 终点领域：朝 goal yaw 对齐
    double goal_yaw = tf2::getYaw(final_goal.pose.orientation);
    double angle_diff = calculateAngleDifference(pose_in_globalframe, goal_yaw);

    // 死区：微小 yaw 误差不输出角速度，抑制足式机器人振动引起的来回微调
    if (std::abs(angle_diff) < YAW_DEADBAND) {
      cmd_vel.twist.angular.z = 0.0;
    } else {
      double yaw_p_gain = 1.0;
      cmd_vel.twist.angular.z = yaw_p_gain * angle_diff;
    }
    if (cmd_vel.twist.angular.z > max_angular_speed_)
      cmd_vel.twist.angular.z = max_angular_speed_;
    if (cmd_vel.twist.angular.z < -max_angular_speed_)
      cmd_vel.twist.angular.z = -max_angular_speed_;

    cmd_vel.twist.linear.x = max_linear_speed_ * vx_ratio;

    // 控制器不做自主刹停，交由 GoalChecker 统一判定到达
  } else {
    // 行进中：Pure Pursuit 曲率控制 + 渐变减速
    double L2 = lx * lx + ly * ly;
    if (L2 < 1e-6) L2 = 1e-6;

    // 前瞻点在机器人后方的保护
    double curvature = (lx > 0.0) ? (2.0 * ly / L2) : 0.0;

    cmd_vel.twist.linear.x = max_linear_speed_ * vx_ratio;
    cmd_vel.twist.angular.z = max_linear_speed_ * curvature;
    if (cmd_vel.twist.angular.z > max_angular_speed_)
      cmd_vel.twist.angular.z = max_angular_speed_;
    if (cmd_vel.twist.angular.z < -max_angular_speed_)
      cmd_vel.twist.angular.z = -max_angular_speed_;
  }

  RCLCPP_INFO(node_->get_logger(),
      "vx=%.2f wz=%.2f | dist=%.2f | %s",
      cmd_vel.twist.linear.x,
      cmd_vel.twist.angular.z, dist_to_final,
      in_final_approach_ ? "ALIGN" : "CRUISE");

  return cmd_vel;
}

void CustomController::setSpeedLimit(const double &, const bool &) {}
void CustomController::setPlan(const nav_msgs::msg::Path &path) { global_plan_ = path; }

geometry_msgs::msg::PoseStamped CustomController::getNearestTargetPose(
    const geometry_msgs::msg::PoseStamped &current_pose) {
  if (global_plan_.poses.size() < 2) return global_plan_.poses.back();
  int nearest_idx = 0;
  double min_dist = 1e9;
  for (size_t i = 0; i < global_plan_.poses.size(); i++) {
    double dist = std::hypot(
        global_plan_.poses[i].pose.position.x - current_pose.pose.position.x,
        global_plan_.poses[i].pose.position.y - current_pose.pose.position.y);
    if (dist < min_dist) { min_dist = dist; nearest_idx = i; }
  }
  double accum_dist = 0.0;
  int target_idx = nearest_idx;
  for (size_t i = nearest_idx; i < global_plan_.poses.size() - 1; i++) {
    accum_dist += std::hypot(
        global_plan_.poses[i+1].pose.position.x - global_plan_.poses[i].pose.position.x,
        global_plan_.poses[i+1].pose.position.y - global_plan_.poses[i].pose.position.y);
    if (accum_dist >= lookahead_dist_) { target_idx = i + 1; break; }
  }
  if (target_idx == nearest_idx && !global_plan_.poses.empty())
    target_idx = global_plan_.poses.size() - 1;
  return global_plan_.poses[target_idx];
}

double CustomController::calculateAngleDifference(
    const geometry_msgs::msg::PoseStamped &current_pose,
    double target_angle) {
  float current_robot_yaw = tf2::getYaw(current_pose.pose.orientation);
  double angle_diff = target_angle - current_robot_yaw;
  if (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
  else if (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  return angle_diff;
}

} // namespace nav2_custom_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)
