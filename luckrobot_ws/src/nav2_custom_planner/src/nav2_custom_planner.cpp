#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>
#include "nav2_core/exceptions.hpp"
#include "nav2_custom_planner/nav2_custom_planner.hpp"

namespace nav2_custom_planner {

void CustomPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                              std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  tf_ = tf; node_ = parent.lock(); name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void CustomPlanner::cleanup() {}
void CustomPlanner::activate() {}
void CustomPlanner::deactivate() {}

nav_msgs::msg::Path CustomPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                              const geometry_msgs::msg::PoseStamped &goal) {
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_)
    throw nav2_core::PlannerException("坐标系错误");

  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  // 计算 start → goal 直线参数
  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;
  double dist = std::hypot(dx, dy);
  double yaw = std::atan2(dy, dx);

  // 沿直线均匀采样，间隔 ≤ interpolation_resolution_
  int num_points = std::max(1, static_cast<int>(std::ceil(dist / interpolation_resolution_)));

  for (int i = 0; i <= num_points; ++i) {
    double t = static_cast<double>(i) / num_points;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = start.pose.position.x + t * dx;
    pose.pose.position.y = start.pose.position.y + t * dy;
    pose.pose.position.z = 0.0;

    // 朝向指向 goal 方向
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);

    path.poses.push_back(pose);
  }

  // 确保首尾精确匹配（消除浮点累积误差），保留原始朝向
  path.poses.front().pose.position = start.pose.position;
  path.poses.front().pose.orientation = start.pose.orientation;
  path.poses.back().pose.position = goal.pose.position;
  path.poses.back().pose.orientation = goal.pose.orientation;

  return path;
}

} // namespace nav2_custom_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner, nav2_core::GlobalPlanner)
