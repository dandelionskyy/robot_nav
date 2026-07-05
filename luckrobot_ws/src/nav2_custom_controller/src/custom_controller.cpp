#include "nav2_custom_controller/custom_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <cmath>

namespace nav2_custom_controller {

const char * CustomController::phaseToString(GoalPhase phase) const {
  switch (phase) {
    case GoalPhase::TRACKING:
      return "TRACKING";
    case GoalPhase::FINAL_ALIGN:
      return "FINAL_ALIGN";
    case GoalPhase::GOAL_REACHED:
      return "GOAL_REACHED";
    default:
      return "UNKNOWN";
  }
}

bool CustomController::isSameFinalGoal(const nav_msgs::msg::Path &new_path) const {
  if (global_plan_.poses.empty() || new_path.poses.empty()) {
    return false;
  }

  const auto &old_goal = global_plan_.poses.back().pose;
  const auto &new_goal = new_path.poses.back().pose;

  double dx = old_goal.position.x - new_goal.position.x;
  double dy = old_goal.position.y - new_goal.position.y;
  double goal_dist = std::hypot(dx, dy);

  geometry_msgs::msg::PoseStamped ref_pose;
  ref_pose.pose = old_goal;
  double old_goal_yaw = tf2::getYaw(ref_pose.pose.orientation);
  double new_goal_yaw = tf2::getYaw(new_goal.orientation);
  double goal_yaw_diff = std::fabs(new_goal_yaw - old_goal_yaw);
  if (goal_yaw_diff > M_PI) {
    goal_yaw_diff = 2.0 * M_PI - goal_yaw_diff;
  }

  return goal_dist <= goal_xy_tolerance_ && goal_yaw_diff <= final_yaw_tolerance_;
}

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
      node_, plugin_name_ + ".max_lateral_speed", rclcpp::ParameterValue(0.1));
  node_->get_parameter(plugin_name_ + ".max_lateral_speed", max_lateral_speed_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(0.7));
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.8));
  node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".positioning_radius", rclcpp::ParameterValue(0.12));
  node_->get_parameter(plugin_name_ + ".positioning_radius", positioning_radius_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".positioning_radius_hysteresis", rclcpp::ParameterValue(0.05));
  node_->get_parameter(plugin_name_ + ".positioning_radius_hysteresis",
                       positioning_radius_hysteresis_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".goal_xy_tolerance", rclcpp::ParameterValue(0.10));
  node_->get_parameter(plugin_name_ + ".goal_xy_tolerance", goal_xy_tolerance_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".final_align_exit_margin", rclcpp::ParameterValue(0.02));
  node_->get_parameter(plugin_name_ + ".final_align_exit_margin", final_align_exit_margin_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".final_yaw_tolerance", rclcpp::ParameterValue(0.25));
  node_->get_parameter(plugin_name_ + ".final_yaw_tolerance", final_yaw_tolerance_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".final_align_hold_heading_error",
      rclcpp::ParameterValue(0.5));
  node_->get_parameter(plugin_name_ + ".final_align_hold_heading_error",
                       final_align_hold_heading_error_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".final_align_kp", rclcpp::ParameterValue(0.4));
  node_->get_parameter(plugin_name_ + ".final_align_kp", final_align_kp_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_final_align_angular_speed",
      rclcpp::ParameterValue(0.25));
  node_->get_parameter(plugin_name_ + ".max_final_align_angular_speed",
                       max_final_align_angular_speed_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".min_positioning_speed", rclcpp::ParameterValue(0.01));
  node_->get_parameter(plugin_name_ + ".min_positioning_speed", min_positioning_speed_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".reverse_enter_angle", rclcpp::ParameterValue(2.2));
  node_->get_parameter(plugin_name_ + ".reverse_enter_angle", reverse_enter_angle_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".reverse_exit_angle", rclcpp::ParameterValue(1.8));
  node_->get_parameter(plugin_name_ + ".reverse_exit_angle", reverse_exit_angle_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".reverse_angular_kp", rclcpp::ParameterValue(0.5));
  node_->get_parameter(plugin_name_ + ".reverse_angular_kp", reverse_angular_kp_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_reverse_speed", rclcpp::ParameterValue(0.08));
  node_->get_parameter(plugin_name_ + ".max_reverse_speed", max_reverse_speed_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_reverse_angular_speed", rclcpp::ParameterValue(0.18));
  node_->get_parameter(plugin_name_ + ".max_reverse_angular_speed", max_reverse_angular_speed_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".reverse_slowdown_angle", rclcpp::ParameterValue(1.2));
  node_->get_parameter(plugin_name_ + ".reverse_slowdown_angle", reverse_slowdown_angle_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".linear_velocity_kd", rclcpp::ParameterValue(0.20));
  node_->get_parameter(plugin_name_ + ".linear_velocity_kd", linear_velocity_kd_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".linear_velocity_delta_limit", rclcpp::ParameterValue(0.04));
  node_->get_parameter(plugin_name_ + ".linear_velocity_delta_limit", linear_velocity_delta_limit_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".allow_reverse_tracking", rclcpp::ParameterValue(true));
  node_->get_parameter(plugin_name_ + ".allow_reverse_tracking", allow_reverse_tracking_);

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".allow_reverse_near_goal", rclcpp::ParameterValue(false));
  node_->get_parameter(plugin_name_ + ".allow_reverse_near_goal", allow_reverse_near_goal_);

  goal_phase_ = GoalPhase::TRACKING;
  last_logged_phase_ = goal_phase_;
  reverse_mode_ = false;
  last_target_linear_x_ = 0.0;
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

  auto target_pose = getNearestTargetPose(pose_in_globalframe);
  auto final_goal = global_plan_.poses.back();

  double dist_to_final = std::hypot(
      final_goal.pose.position.x - pose_in_globalframe.pose.position.x,
      final_goal.pose.position.y - pose_in_globalframe.pose.position.y);

  double path_angle_diff =
      calculateAngleDifference(pose_in_globalframe, target_pose);
  double goal_yaw = tf2::getYaw(final_goal.pose.orientation);
  double goal_angle_diff = calculateAngleDifference(pose_in_globalframe, goal_yaw);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = node_->get_clock()->now();

  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.linear.y = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  GoalPhase phase_before_update = goal_phase_;

  if (goal_phase_ == GoalPhase::GOAL_REACHED) {
    if (dist_to_final > goal_xy_tolerance_ + positioning_radius_hysteresis_ ||
        std::fabs(goal_angle_diff) > final_yaw_tolerance_ + 0.05) {
      goal_phase_ = GoalPhase::TRACKING;
    }
  }

  double final_align_exit_dist = goal_xy_tolerance_ + final_align_exit_margin_;
  double final_align_hold_dist = positioning_radius_ + positioning_radius_hysteresis_;
  bool keep_final_align_for_heading =
      std::fabs(goal_angle_diff) > final_align_hold_heading_error_ &&
      dist_to_final <= final_align_hold_dist;

  if (goal_phase_ == GoalPhase::TRACKING && dist_to_final <= positioning_radius_) {
    goal_phase_ = GoalPhase::FINAL_ALIGN;
  } else if (goal_phase_ == GoalPhase::FINAL_ALIGN &&
             dist_to_final > final_align_exit_dist &&
             !keep_final_align_for_heading) {
    goal_phase_ = GoalPhase::TRACKING;
  }

  if (dist_to_final <= goal_xy_tolerance_ &&
      std::fabs(goal_angle_diff) <= final_yaw_tolerance_) {
    goal_phase_ = GoalPhase::GOAL_REACHED;
  }

  double target_linear_x = 0.0;
  if (goal_phase_ == GoalPhase::TRACKING) {
    double abs_path_angle_diff = std::fabs(path_angle_diff);
    if (allow_reverse_tracking_) {
      if (!reverse_mode_ && abs_path_angle_diff >= reverse_enter_angle_) {
        reverse_mode_ = true;
      } else if (reverse_mode_ && abs_path_angle_diff <= reverse_exit_angle_) {
        reverse_mode_ = false;
      }
    } else {
      reverse_mode_ = false;
    }

    if (reverse_mode_) {
      double target_heading = std::atan2(
          target_pose.pose.position.y - pose_in_globalframe.pose.position.y,
          target_pose.pose.position.x - pose_in_globalframe.pose.position.x);
      double rear_heading = tf2::getYaw(pose_in_globalframe.pose.orientation) + M_PI;
      if (rear_heading > M_PI) {
        rear_heading -= 2.0 * M_PI;
      }
      double reverse_heading_error = target_heading - rear_heading;
      if (reverse_heading_error < -M_PI) {
        reverse_heading_error += 2.0 * M_PI;
      } else if (reverse_heading_error > M_PI) {
        reverse_heading_error -= 2.0 * M_PI;
      }

      double reverse_speed_scale = 1.0;
      if (std::fabs(reverse_heading_error) > reverse_slowdown_angle_) {
        reverse_speed_scale = std::clamp(
            1.0 - (std::fabs(reverse_heading_error) - reverse_slowdown_angle_) /
                      std::max(M_PI - reverse_slowdown_angle_, 1e-3),
            0.2,
            1.0);
      }
      target_linear_x = -max_reverse_speed_ * reverse_speed_scale;
      cmd_vel.twist.angular.z = std::clamp(
          reverse_angular_kp_ * reverse_heading_error,
          -max_reverse_angular_speed_,
          max_reverse_angular_speed_);
    } else {
      double linear_factor = std::cos(path_angle_diff);
      if (linear_factor < 0.0) {
        linear_factor = 0.0;
      }

      target_linear_x = max_linear_speed_ * linear_factor;
      cmd_vel.twist.angular.z = 0.5 * std::sin(path_angle_diff);
    }

    if (dist_to_final <= positioning_radius_ + positioning_radius_hysteresis_) {
      double near_goal_radius = positioning_radius_ + positioning_radius_hysteresis_;
      double ratio = std::clamp(dist_to_final / std::max(near_goal_radius, 1e-3), 0.0, 1.0);
      double speed_limit = reverse_mode_ ? max_reverse_speed_ : max_linear_speed_;
      double scaled_speed = speed_limit * std::pow(ratio, 2.0);
      target_linear_x = std::copysign(
          std::max(std::fabs(scaled_speed), min_positioning_speed_), target_linear_x);

      if (!allow_reverse_near_goal_ && target_linear_x < 0.0) {
        target_linear_x = 0.0;
        reverse_mode_ = false;
      }

      if (reverse_mode_) {
        double target_heading = std::atan2(
            target_pose.pose.position.y - pose_in_globalframe.pose.position.y,
            target_pose.pose.position.x - pose_in_globalframe.pose.position.x);
        double rear_heading = tf2::getYaw(pose_in_globalframe.pose.orientation) + M_PI;
        if (rear_heading > M_PI) {
          rear_heading -= 2.0 * M_PI;
        }
        double reverse_heading_error = target_heading - rear_heading;
        if (reverse_heading_error < -M_PI) {
          reverse_heading_error += 2.0 * M_PI;
        } else if (reverse_heading_error > M_PI) {
          reverse_heading_error -= 2.0 * M_PI;
        }
        cmd_vel.twist.angular.z = std::clamp(
            0.30 * reverse_heading_error,
            -max_reverse_angular_speed_,
            max_reverse_angular_speed_);
      } else {
        cmd_vel.twist.angular.z = 0.35 * std::sin(path_angle_diff);
      }
    }

    double velocity_delta = target_linear_x - last_target_linear_x_;
    double damping = linear_velocity_kd_ * velocity_delta;
    cmd_vel.twist.linear.x = target_linear_x - damping;
    double max_delta = std::max(linear_velocity_delta_limit_, 1e-3);
    cmd_vel.twist.linear.x = std::clamp(
        cmd_vel.twist.linear.x,
        last_target_linear_x_ - max_delta,
        last_target_linear_x_ + max_delta);
    if ((target_linear_x > 0.0 && cmd_vel.twist.linear.x < 0.0) ||
        (target_linear_x < 0.0 && cmd_vel.twist.linear.x > 0.0)) {
      cmd_vel.twist.linear.x = 0.0;
    }
    last_target_linear_x_ = cmd_vel.twist.linear.x;

    cmd_vel.twist.angular.z = std::clamp(
        cmd_vel.twist.angular.z, -max_angular_speed_, max_angular_speed_);
    // Track-mode steering follows the chassis' non-ROS yaw sign convention.
    cmd_vel.twist.angular.z = -cmd_vel.twist.angular.z;
  } else if (goal_phase_ == GoalPhase::FINAL_ALIGN) {
    last_target_linear_x_ = 0.0;
    if (std::fabs(goal_angle_diff) > final_yaw_tolerance_) {
      cmd_vel.twist.angular.z = std::clamp(
          final_align_kp_ * goal_angle_diff,
          -max_final_align_angular_speed_,
          max_final_align_angular_speed_);
    }
  } else {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
  }

  if (goal_phase_ != phase_before_update || goal_phase_ != last_logged_phase_) {
    RCLCPP_INFO(
        node_->get_logger(),
        "[GoalPhase] %s -> %s | dist=%.3f m goal_yaw_err=%.3f rad "
        "(enter=%.3f exit=%.3f xy_tol=%.3f yaw_tol=%.3f hold_heading=%.3f hold_dist=%.3f)",
        phaseToString(phase_before_update),
        phaseToString(goal_phase_),
        dist_to_final,
        goal_angle_diff,
        positioning_radius_,
        final_align_exit_dist,
        goal_xy_tolerance_,
        final_yaw_tolerance_,
        final_align_hold_heading_error_,
        final_align_hold_dist);
    last_logged_phase_ = goal_phase_;
  }

  RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "[NavCtrl] phase=%s dist=%.3f m path_err=%.3f rad goal_err=%.3f rad "
      "reverse=%s target_vx=%.3f cmd(vx=%.3f, wz=%.3f)",
      phaseToString(goal_phase_),
      dist_to_final,
      path_angle_diff,
      goal_angle_diff,
      reverse_mode_ ? "true" : "false",
      target_linear_x,
      cmd_vel.twist.linear.x,
      cmd_vel.twist.angular.z);

  return cmd_vel;
}

void CustomController::setSpeedLimit(const double &, const bool &) {}
void CustomController::setPlan(const nav_msgs::msg::Path &path) {
  bool same_goal = isSameFinalGoal(path);
  GoalPhase phase_before_plan = goal_phase_;
  global_plan_ = path;
  reverse_mode_ = false;
  last_target_linear_x_ = 0.0;

  if (!same_goal || phase_before_plan == GoalPhase::TRACKING) {
    goal_phase_ = GoalPhase::TRACKING;
    last_logged_phase_ = goal_phase_;
  }

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[NavCtrl] Received empty plan");
    return;
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "[NavCtrl] New plan accepted | poses=%zu final_goal=(%.3f, %.3f, yaw=%.3f) "
      "same_goal=%s phase=%s->%s",
      global_plan_.poses.size(),
      global_plan_.poses.back().pose.position.x,
      global_plan_.poses.back().pose.position.y,
      tf2::getYaw(global_plan_.poses.back().pose.orientation),
      same_goal ? "true" : "false",
      phaseToString(phase_before_plan),
      phaseToString(goal_phase_));
}

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
    const geometry_msgs::msg::PoseStamped &target_pose) {
  double current_robot_yaw = tf2::getYaw(current_pose.pose.orientation);
  double target_angle = std::atan2(
      target_pose.pose.position.y - current_pose.pose.position.y,
      target_pose.pose.position.x - current_pose.pose.position.x);
  double angle_diff = target_angle - current_robot_yaw;
  if (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
  else if (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  return angle_diff;
}

double CustomController::calculateAngleDifference(
    const geometry_msgs::msg::PoseStamped &current_pose,
    double target_angle) {
  double current_robot_yaw = tf2::getYaw(current_pose.pose.orientation);
  double angle_diff = target_angle - current_robot_yaw;
  if (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
  else if (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  return angle_diff;
}

} // namespace nav2_custom_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)
