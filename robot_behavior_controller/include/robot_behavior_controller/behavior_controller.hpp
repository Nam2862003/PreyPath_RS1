#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <deque>

// Include FollowWaypoints action
#include <nav2_msgs/action/follow_waypoints.hpp>

namespace robot_behavior_controller {

enum class RobotMode { IDLE, TRAVERSING, MANUAL, ESTOPPED, PATROLLING };

class BehaviorController : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleWps = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  BehaviorController();

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr traverse_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rtb_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_enable_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr inspect_len_sub_;
  // Split status output into two topics:
  //  - status_mode_pub_: publishes current robot mode as a plain string (e.g., "IDLE") on /behavior/status
  //  - comms_pub_: publishes human-readable comments/messages on /behavior/comms
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr comms_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr wps_client_;

  RobotMode mode_{RobotMode::IDLE};
  GoalHandleNav::SharedPtr active_goal_;
  // Inspection state
  double pending_inspect_L_{0.0};
  bool inspecting_{false};
  geometry_msgs::msg::PoseStamped last_traverse_goal_{}; // center of square
  std::deque<geometry_msgs::msg::PoseStamped> waypoint_queue_{};

  void traverseGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void returnBaseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void manualEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void manualCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void inspectLengthCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void publishZeroTwist();

  void sendTraverseGoal(const geometry_msgs::msg::PoseStamped &pose);
  // New split publishers
  void publishMode();
  void publishComms(const std::string &text);
  std::string modeString() const;
  void sendWaypointList(const std::vector<geometry_msgs::msg::PoseStamped> &poses);

  // Internal helpers for inspection
  void startInspectionIfRequested();
  void clearInspection(const char *reason = nullptr);
};

} // namespace robot_behavior_controller
