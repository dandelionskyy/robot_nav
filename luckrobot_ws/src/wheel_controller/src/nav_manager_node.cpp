#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <map>
#include <string>
#include <cmath>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// 路点：底盘(X,Y,Yaw) + 模式编号
struct Waypoint {
    double x;
    double y;
    double yaw;
    uint8_t mode;
};

class NavManagerNode : public rclcpp::Node {
public:
    NavManagerNode() : Node("nav_manager_node") {

        // 10个预设模式航点 (坐标按实际修改)
        target_locations_["模式0"] = { 0.0,  0.0,  0.,   0 };
        target_locations_["模式1"] = { 2.0,  2.0,  0.00, 1 };
        target_locations_["模式2"] = { 3.0,  3.0,  3.00, 2 };
        target_locations_["模式3"] = { 5.0, -2.0, -1.57, 3 };
        target_locations_["模式4"] = { 5.0, -3.0, -1.57, 4 };
        target_locations_["模式5"] = {-4.0, -4.0,  3.14, 5 };
        target_locations_["模式6"] = { 1.5,  4.2,  1.57, 6 };
        target_locations_["模式7"] = { 3.2, -1.5,  0.00, 7 };
        target_locations_["模式8"] = { 6.1,  0.5,  1.57, 8 };
        target_locations_["模式9"] = { 6.1,  2.5,  1.57, 9 };

        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // 订阅 /cmd_mode (UInt8)，收到 mode 编号触发对应航点
        mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/cmd_mode", 10,
            std::bind(&NavManagerNode::mode_callback, this, std::placeholders::_1)
        );

        // 发布 /cmd_vel_mode，导航到达后发送 [0,0,0,mode]
        cmd_vel_mode_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/cmd_vel_mode", 10);

        RCLCPP_INFO(this->get_logger(), "导航调度节点启动成功 (10模式, /cmd_mode → 航点 → /cmd_vel_mode)");
    }

private:
    std::map<std::string, Waypoint> target_locations_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_vel_mode_pub_;

    uint8_t current_mode_ = 0;

    void yaw_to_quaternion(double yaw, double& qx, double& qy, double& qz, double& qw) {
        qx = 0.0; qy = 0.0;
        qz = std::sin(yaw / 2.0);
        qw = std::cos(yaw / 2.0);
    }

    void mode_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        uint8_t mode = msg->data;
        RCLCPP_INFO(this->get_logger(), "收到模式指令: %d", mode);

        // 查找对应航点
        std::string target_name = "模式" + std::to_string(mode);
        auto it = target_locations_.find(target_name);
        if (it == target_locations_.end()) {
            RCLCPP_WARN(this->get_logger(), "未知模式 %d，忽略", mode);
            return;
        }

        Waypoint wp = it->second;
        current_mode_ = wp.mode;
        RCLCPP_INFO(this->get_logger(), "目标: [%s] X=%.2f Y=%.2f Yaw=%.2f Mode=%d",
                    target_name.c_str(), wp.x, wp.y, wp.yaw, wp.mode);

        send_nav_goal(wp.x, wp.y, wp.yaw);
    }

    void send_nav_goal(double x, double y, double yaw) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action 服务器未就绪！无法导航！");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;

        double qx, qy, qz, qw;
        yaw_to_quaternion(yaw, qx, qy, qz, qw);
        goal_msg.pose.pose.orientation.x = qx;
        goal_msg.pose.pose.orientation.y = qy;
        goal_msg.pose.pose.orientation.z = qz;
        goal_msg.pose.pose.orientation.w = qw;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.result_callback =
            [this](const GoalHandleNav::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                    {
                        RCLCPP_INFO(this->get_logger(), "导航到达！发布 /cmd_vel_mode: mode=%d", current_mode_);

                        std_msgs::msg::Float64MultiArray mode_msg;
                        mode_msg.data = {0.0, 0.0, 0.0, static_cast<double>(current_mode_)};
                        cmd_vel_mode_pub_->publish(mode_msg);
                        break;
                    }
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "导航被强行终止！(障碍物死锁)");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "导航任务被外部取消。");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "导航失败，未知原因！");
                        break;
                }
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "路线已下发至 Nav2，机器人开始移动...");
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavManagerNode>());
    rclcpp::shutdown();
    return 0;
}
