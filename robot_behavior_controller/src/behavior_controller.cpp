#include "robot_behavior_controller/behavior_controller.hpp"
#include "robot_behavior_controller/patrolling_waypoints.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace robot_behavior_controller {

BehaviorController::BehaviorController()
  : rclcpp::Node("behavior_controller")   // Create ROS 2 node with a fixed name.
{
  // STATUS PUBLISHERS --------------------------------------------------------------
  status_mode_pub_ = this->create_publisher<std_msgs::msg::String>("/behavior/status", 10);
  comms_pub_ = this->create_publisher<std_msgs::msg::String>("/behavior/comms", 10);

  // TRAVERSE GOAL SUBSCRIPTION ----------------------------------------------------
  traverse_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/behavior/traverse_goal", 10,
      std::bind(&BehaviorController::traverseGoalCallback, this, std::placeholders::_1));

  // RETURN-TO-BASE SUBSCRIPTION ---------------------------------------------------
  rtb_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/behavior/return_to_base", 10,
      std::bind(&BehaviorController::returnBaseCallback, this, std::placeholders::_1));

  // MANUAL MODE SUBSCRIPTIONS -----------------------------------------------------
  manual_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/behavior/manual_enable", 10,
      std::bind(&BehaviorController::manualEnableCallback, this, std::placeholders::_1));

  manual_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/behavior/manual_cmd", 10,
      std::bind(&BehaviorController::manualCmdCallback, this, std::placeholders::_1));

  // INSPECTION LENGTH SUBSCRIPTION -----------------------------------------------
  inspect_len_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/behavior/inspect_length", 10,
      std::bind(&BehaviorController::inspectLengthCallback, this, std::placeholders::_1));

  // MANUAL VELOCITY OUTPUT --------------------------------------------------------
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // E-STOP SUBSCRIPTION -----------------------------------------------------------
  estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/estop", 10,
      std::bind(&BehaviorController::estopCallback, this, std::placeholders::_1));

  // NAV2 ACTION CLIENTS -----------------------------------------------------------
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "navigate_to_pose");

  wps_client_ = rclcpp_action::create_client<FollowWaypoints>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "follow_waypoints");

  // HUMAN DETECTION SUBSCRIPTION --------------------------------------------------
  human_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/human_pose", 10,
      std::bind(&BehaviorController::detectionCallback, this, std::placeholders::_1));

  // INITIAL STATUS ---------------------------------------------------------------
  publishComms("BehaviorController started");
  publishMode();
}

//
// Helper: publish mode and comments via separate topics.
//
void BehaviorController::publishMode() {
  if (!status_mode_pub_) return;
  std_msgs::msg::String msg;
  msg.data = modeString();
  status_mode_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Mode=%s", msg.data.c_str());
}

void BehaviorController::publishComms(const std::string &text) {
  if (!comms_pub_) return;
  std_msgs::msg::String msg;
  msg.data = text;
  comms_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "%s", text.c_str());
}

std::string BehaviorController::modeString() const {
  switch (mode_) {
    case RobotMode::IDLE:        return "IDLE";
    case RobotMode::PATROLLING:  return "PATROLLING";
    case RobotMode::TRAVERSING:  return "TRAVERSING";
    case RobotMode::MANUAL:      return "MANUAL";
    case RobotMode::ESTOPPED:    return "ESTOPPED";
  }
  return "UNKNOWN";
}

//
// E-STOP logic
//
void BehaviorController::estopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg || !msg->data) return;
  if (active_goal_) {
    nav_client_->async_cancel_goal(active_goal_);
    active_goal_.reset();
  }
  if (mode_ == RobotMode::TRAVERSING) mode_ = RobotMode::IDLE;
  publishZeroTwist();
  clearInspection("E-STOP");
  publishComms("E-STOP pressed: goals canceled, motion halted");
  publishMode();
}

//
// Traverse goal callback
//
void BehaviorController::traverseGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!msg) return;
  if (mode_ == RobotMode::MANUAL) {
    publishComms("Traverse goal ignored -> MANUAL");
    return;
  }

  // Reset the human detection trigger — new mission started
  human_abort_triggered_ = false;

  publishComms("Received traverse goal: (" +
               std::to_string(msg->pose.position.x) + ", " +
               std::to_string(msg->pose.position.y) + ")");
  last_traverse_goal_ = *msg;
  sendTraverseGoal(*msg);
}

//
// Return-to-base callback
//
void BehaviorController::returnBaseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!msg) return;
  if (mode_ == RobotMode::MANUAL) {
    publishComms("RTB goal ignored -> MANUAL");
    return;
  }

  // Reset human detection flag for a new mission
  human_abort_triggered_ = false;

  publishComms("Received RTB goal: (" +
               std::to_string(msg->pose.position.x) + ", " +
               std::to_string(msg->pose.position.y) + ")");
  sendTraverseGoal(*msg);
}

//
// Manual enable/disable
//
void BehaviorController::manualEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    if (mode_ != RobotMode::MANUAL) {
      if (active_goal_) {
        nav_client_->async_cancel_goal(active_goal_);
        active_goal_.reset();
      }
      mode_ = RobotMode::MANUAL;
      publishComms("Manual mode enabled");
      publishZeroTwist();
      clearInspection("Manual enabled");
      publishMode();
    }
  } else {
    if (mode_ == RobotMode::MANUAL) {
      publishZeroTwist();
      mode_ = RobotMode::IDLE;
      publishComms("Manual mode disabled");
      publishMode();
    }
  }
}

//
// Manual velocity command
//
void BehaviorController::manualCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (mode_ != RobotMode::MANUAL) return;
  if (cmd_vel_pub_) cmd_vel_pub_->publish(*msg);
}

//
// Publish zero twist
//
void BehaviorController::publishZeroTwist() {
  if (!cmd_vel_pub_) return;
  geometry_msgs::msg::Twist z;
  cmd_vel_pub_->publish(z);
}

//
// Send a Nav2 traverse goal
//
void BehaviorController::sendTraverseGoal(const geometry_msgs::msg::PoseStamped &pose) {
  if (!nav_client_->wait_for_action_server(500ms)) {
    publishComms("Nav2 action server not available");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = pose;

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  opts.goal_response_callback = [this](std::shared_ptr<GoalHandleNav> handle) {
    if (!handle) {
      publishComms("Traverse goal rejected by server");
    }
    else if (human_abort_triggered_) {
      publishComms("Traverse/patrolling goal aborted due to human detection");
      mode_ = RobotMode::TRAVERSING;
      publishMode();
    } else {
      active_goal_ = handle;
      mode_ = inspecting_ ? RobotMode::PATROLLING : RobotMode::TRAVERSING;
      publishComms("Traverse goal accepted");
      publishMode();
    }
  };

  opts.result_callback = [this](const GoalHandleNav::WrappedResult &res) {
    std::string outcome;
    switch (res.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: outcome = "SUCCEEDED"; break;
      case rclcpp_action::ResultCode::ABORTED:   outcome = "ABORTED"; break;
      case rclcpp_action::ResultCode::CANCELED:  outcome = "CANCELED"; break;
      default: outcome = "UNKNOWN"; break;
    }
    active_goal_.reset();
    mode_ = RobotMode::IDLE;
    publishComms("Traverse result: " + outcome);
    publishMode();

    // Reset abort flag once mission is done
    human_abort_triggered_ = false;

    if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
      startInspectionIfRequested();
    }
  };

  nav_client_->async_send_goal(goal, opts);
}

//
// INSPECTION FLOW
//
void BehaviorController::inspectLengthCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  if (!msg) return;
  pending_inspect_L_ = std::max(0.0, msg->data);
  if (pending_inspect_L_ > 0.0) {
    publishComms("Inspection requested: L=" + std::to_string(pending_inspect_L_) + " m");
    // Reset abort flag for a new mission
    human_abort_triggered_ = false;
  }
}

void BehaviorController::startInspectionIfRequested() {
  if (pending_inspect_L_ <= 0.0) return;
  auto waypoints = generate_patrol_waypoints(last_traverse_goal_, pending_inspect_L_);
  pending_inspect_L_ = 0.0;
  if (waypoints.empty()) {
    publishComms("Inspection requested but no waypoints generated");
    return;
  }
  publishComms("Patrolling in progress. Waypoints=" + std::to_string(waypoints.size()));
  sendWaypointList(waypoints);
}

void BehaviorController::sendWaypointList(const std::vector<geometry_msgs::msg::PoseStamped> &poses) {
  if (!wps_client_ || !wps_client_->wait_for_action_server(500ms)) {
    publishComms("FollowWaypoints server not available; cannot start patrolling");
    return;
  }

  FollowWaypoints::Goal goal;
  goal.poses = poses;

  auto opts = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  opts.goal_response_callback = [this](std::shared_ptr<GoalHandleWps> handle) {
    if (!handle) {
      publishComms("Patrolling goal rejected");
    } else {
      active_wps_goal_ = handle;
      mode_ = RobotMode::PATROLLING;
      publishComms("Patrolling in progress");
      publishMode();
    }
  };

  opts.result_callback = [this](const GoalHandleWps::WrappedResult &res) {
    std::string outcome;
    switch (res.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: outcome = "SUCCEEDED"; break;
      case rclcpp_action::ResultCode::ABORTED:   outcome = "ABORTED"; break;
      case rclcpp_action::ResultCode::CANCELED:  outcome = "CANCELED"; break;
      default: outcome = "UNKNOWN"; break;
    }
    mode_ = RobotMode::IDLE;
    publishComms("Patrolling result: " + outcome);
    publishMode();
    clearInspection("Patrolling done");
    human_abort_triggered_ = false;

    if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
      publishComms("Returning to base (0,0)");
      geometry_msgs::msg::PoseStamped base_pose;
      base_pose.header.frame_id = "map";
      base_pose.header.stamp = this->now();
      base_pose.pose.position.x = 0.0;
      base_pose.pose.position.y = 0.0;
      base_pose.pose.orientation.z = 0.0;
      base_pose.pose.orientation.w = 1.0;
      sendTraverseGoal(base_pose);
    }
  };

  wps_client_->async_send_goal(goal, opts);
}

void BehaviorController::clearInspection(const char *reason) {
  inspecting_ = false;
  waypoint_queue_.clear();
  pending_inspect_L_ = 0.0;
  if (reason) publishComms(std::string("Inspection cleared: ") + reason);
}

//
// HUMAN DETECTION CALLBACK — triggers only once per mission
//
void BehaviorController::detectionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (!msg || human_abort_triggered_) return;  // ignore if already triggered

  // Mark that we’ve already reacted to this detection
  human_abort_triggered_ = true;


  // Cancel NavigateToPose goal if active
  if (active_goal_) {
    try { nav_client_->async_cancel_goal(active_goal_); } catch (...) {}
    active_goal_.reset();
  }

  // Cancel FollowWaypoints goal if active
  if (active_wps_goal_) {
    try { wps_client_->async_cancel_goal(active_wps_goal_); } catch (...) {}
    active_wps_goal_.reset();
  }

  clearInspection("Human detected");

  // Return to base (0,0)
  publishComms("Human detected -> returning to base (0,0)");
  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.header.frame_id = "map";
  base_pose.header.stamp = this->now();
  base_pose.pose.position.x = 0.0;
  base_pose.pose.position.y = 0.0;
  base_pose.pose.orientation.z = 0.0;
  base_pose.pose.orientation.w = 1.0;
  sendTraverseGoal(base_pose);
}

} // namespace robot_behavior_controller


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_behavior_controller::BehaviorController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
