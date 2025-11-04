#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <deque>
#include <vector>

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
  // ------------------- Subscriptions -------------------
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr traverse_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rtb_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_enable_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr inspect_len_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr human_pose_sub_;

  // ------------------- Publishers -------------------
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr comms_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ------------------- Action Clients -------------------
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr wps_client_;

  // ------------------- State Variables -------------------
  RobotMode mode_{RobotMode::IDLE};
  GoalHandleNav::SharedPtr active_goal_;
  GoalHandleWps::SharedPtr active_wps_goal_;
  bool human_abort_triggered_{false};  // true once mission aborted
  bool human_comms_sent_{false};       // ensures comms/status update only once

  // Inspection state
  double pending_inspect_L_{0.0};
  bool inspecting_{false};
  geometry_msgs::msg::PoseStamped last_traverse_goal_{};
  std::deque<geometry_msgs::msg::PoseStamped> waypoint_queue_{};

  // ------------------- Callbacks -------------------
  void traverseGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void returnBaseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void manualEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void manualCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void inspectLengthCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void detectionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void publishZeroTwist();

  // ------------------- Core Behaviors -------------------
  void sendTraverseGoal(const geometry_msgs::msg::PoseStamped &pose);
  void sendWaypointList(const std::vector<geometry_msgs::msg::PoseStamped> &poses);
  void startInspectionIfRequested();
  void clearInspection(const char *reason = nullptr);

  // ------------------- Status & Logging -------------------
  void publishMode();
  void publishComms(const std::string &text);
  std::string modeString() const;
};

}  // namespace robot_behavior_controller
