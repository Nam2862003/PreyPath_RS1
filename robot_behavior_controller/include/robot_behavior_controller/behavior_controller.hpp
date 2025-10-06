#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace robot_behavior_controller {

enum class RobotMode { IDLE, TRAVERSING, MANUAL, ESTOPPED };

class BehaviorController : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  BehaviorController();

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr traverse_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rtb_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_enable_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  RobotMode mode_{RobotMode::IDLE};
  GoalHandleNav::SharedPtr active_goal_;

  void traverseGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void returnBaseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void manualEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void manualCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publishZeroTwist();

  void sendTraverseGoal(const geometry_msgs::msg::PoseStamped &pose);
  void publishStatus(const std::string &text);
  std::string modeString() const;
};

} // namespace robot_behavior_controller
