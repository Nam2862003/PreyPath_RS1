/**
 * @file behavior_controller.cpp
 * @brief Simple behavior/state controller for the robot.
 *
 * Responsibilities (current):
 *  - Accept external "traverse" (navigation) goals via a PoseStamped topic.
 *  - Forward them to Nav2 (navigate_to_pose action) and track goal lifecycle.
 *  - Maintain and publish a simple internal robot mode (IDLE / TRAVERSING / ESTOPPED).
 *  - React to an emergency stop (e‑stop) topic and cancel active goals.
 *  - Publish human readable status updates for UI panels or logs.
 *
 * Design notes:
 *  - Kept intentionally minimal so you can extend with more modes (PATROL, DOCK, INSPECT, etc).
 *  - Uses enum RobotMode for clarity and future scalability.
 *  - Wraps all status text through publishStatus() for consistent logging + topic publishing.
 *  - Does NOT (yet) expose services for querying mode; could be added later.
 *  - Orientation of incoming traverse goals is passed through as-is (UI currently sets yaw=0).
 */

#include "robot_behavior_controller/behavior_controller.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace robot_behavior_controller {

BehaviorController::BehaviorController()
  : rclcpp::Node("behavior_controller")   // Create ROS 2 node with a fixed name.
{
  // Publisher for human-readable status text.
  // UI (RViz panel) subscribes to this to show behavior / error messages.
  // QoS depth 10 is enough; no latching needed, we just stream state changes.
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/behavior/status", 10);

  // Subscription: traverse goals (PoseStamped). These are high-level navigation targets
  // produced by the UI (or other planners later). Topic is intentionally in the
  // behavior namespace to keep ownership clear.
  traverse_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/behavior/traverse_goal",
      10,
      std::bind(&BehaviorController::traverseGoalCallback, this, std::placeholders::_1));

  // Subscription: return-to-base pose (often same as 'home'). Separate topic so UI
  // or higher-level autonomy can choose which semantic command to invoke.
  rtb_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/behavior/return_to_base",
    10,
    std::bind(&BehaviorController::returnBaseCallback, this, std::placeholders::_1));

  // Subscription: e-stop command. A Bool true means "enter ESTOPPED mode".
  // False releases the e-stop (returns to IDLE if previously ESTOPPED).
  estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/estop",
      10,
      std::bind(&BehaviorController::estopCallback, this, std::placeholders::_1));

  // Nav2 NavigateToPose action client.
  // IMPORTANT: We cannot call shared_from_this() inside the constructor because the
  // shared_ptr that owns 'this' isn't fully constructed yet. Therefore we use the
  // interface-based overload of create_client.
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "navigate_to_pose"); // Standard Nav2 action name.

  // Initial status broadcast so UI immediately shows something meaningful.
  publishStatus("BehaviorController started. Mode=" + modeString());
}

//
// Helper: publish a status string to both the topic and the node logger.
//
void BehaviorController::publishStatus(const std::string &text) {
  std_msgs::msg::String msg;
  msg.data = text;
  status_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "%s", text.c_str());
}

//
// Helper: convert current RobotMode to readable string.
//
std::string BehaviorController::modeString() const {
  switch (mode_) {
    case RobotMode::IDLE:        return "IDLE";
    case RobotMode::TRAVERSING:  return "TRAVERSING";
    case RobotMode::ESTOPPED:    return "ESTOPPED";
  }
  return "UNKNOWN"; // Fallback (should never happen).
}

//
// Callback: E-Stop state changed.
//  - If true: enter ESTOPPED, cancel any active goal.
//  - If false and we WERE estopped: revert to IDLE.
//    (Does not auto-resume prior goal; more advanced logic could restore state.)
//
void BehaviorController::estopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    mode_ = RobotMode::ESTOPPED;
    publishStatus("E-STOP engaged. Robot halted.");
    // Cancel any active Nav2 goal (if server is still up).
    if (active_goal_) {
      nav_client_->async_cancel_goal(active_goal_);
      active_goal_.reset();
    }
  } else {
    if (mode_ == RobotMode::ESTOPPED) {
      mode_ = RobotMode::IDLE;
      publishStatus("E-STOP released. Mode=" + modeString());
    }
  }
}

//
// Callback: A new traverse goal (PoseStamped) was received.
//  - Reject if in ESTOPPED mode.
//  - Otherwise forward to Nav2 via sendTraverseGoal().
//
void BehaviorController::traverseGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (mode_ == RobotMode::ESTOPPED) {
    publishStatus("Rejecting traverse goal: ESTOPPED");
    return;
  }

  // (Optional future check) If already TRAVERSING you might decide to:
  //  - Cancel and replace
  //  - Queue
  //  - Ignore new goal
  // For now: always accept and forward (Nav2 can handle multiple requests but we track one).
  publishStatus("Received traverse goal: (" +
                std::to_string(msg->pose.position.x) + ", " +
                std::to_string(msg->pose.position.y) + ")");
  sendTraverseGoal(*msg);
}

// Callback: Return-to-base command received (PoseStamped of home/base).
// For now it's identical handling to traverse; kept separate for future logic
// (e.g., docking behaviors, alignment constraints, pre/post actions).
void BehaviorController::returnBaseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (mode_ == RobotMode::ESTOPPED) {
    publishStatus("Rejecting RTB goal: ESTOPPED");
    return;
  }
  publishStatus("Received RTB goal: (" + std::to_string(msg->pose.position.x) + ", " +
                std::to_string(msg->pose.position.y) + ")");
  sendTraverseGoal(*msg); // Reuse same sending logic
}

//
// Internal: Forward a goal to Nav2 NavigateToPose action.
//  - Wait briefly for server.
//  - Populate goal.
//  - Set up callbacks for acceptance and result.
//  - Does not (yet) provide feedback callback; can be added later.
//
void BehaviorController::sendTraverseGoal(const geometry_msgs::msg::PoseStamped &pose) {
  // Brief wait (500ms) for server. In production you might:
  //  - Retry longer with backoff
  //  - Transition to a degraded mode
  if (!nav_client_->wait_for_action_server(500ms)) {
    publishStatus("Nav2 action server not available");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = pose; // Copy entire PoseStamped (header + pose).

  // Configure asynchronous goal send options.
  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // Called once Nav2 decides to accept or reject the goal.
  opts.goal_response_callback = [this](std::shared_ptr<GoalHandleNav> handle) {
    if (!handle) {
      publishStatus("Traverse goal rejected by server");
    } else {
      active_goal_ = handle;
      mode_ = RobotMode::TRAVERSING;
      publishStatus("Traverse goal accepted. Mode=" + modeString());
    }
  };

  // Called when the goal finishes (success / abort / cancel).
  opts.result_callback = [this](const GoalHandleNav::WrappedResult &res) {
    std::string outcome;
    switch (res.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: outcome = "SUCCEEDED"; break;
      case rclcpp_action::ResultCode::ABORTED:   outcome = "ABORTED";   break;
      case rclcpp_action::ResultCode::CANCELED:  outcome = "CANCELED";  break;
      default:                                   outcome = "UNKNOWN";   break;
    }
    active_goal_.reset();
    // Simplistic: always return to IDLE (even if ESTOP triggered mid-way we would
    // have canceled earlier). If you add more modes, handle transitions carefully.
    mode_ = RobotMode::IDLE;
    publishStatus("Traverse result: " + outcome + ". Mode=" + modeString());
  };

  // (Optional future) opts.feedback_callback = [...] to stream progress.

  nav_client_->async_send_goal(goal, opts);
}

} // namespace robot_behavior_controller

//
// Standard ROS 2 entry point.
//
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_behavior_controller::BehaviorController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}