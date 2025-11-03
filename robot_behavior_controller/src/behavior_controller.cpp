/**
 * @file behavior_controller.cpp
 *  1. Constructor: sets up all publishers / subscribers / action client.
 *  2. publishStatus(): unified logging + topic publication.
 *  3. estopCallback(): momentary cancel + halt semantics.
 *  4. traverseGoalCallback() & returnBaseCallback(): accept PoseStamped goals and forward.
 *  5. manualEnableCallback(): toggles MANUAL mode (cancels autonomous goal if needed).
 *  6. manualCmdCallback(): forwards Twist when in MANUAL.
 *  7. sendTraverseGoal(): wraps Nav2 action client logic and state transitions.
 */

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
  // Split into two channels:
  //  * /behavior/status : current mode as a plain string (e.g., "IDLE")
  //  * /behavior/comms  : human‑readable comments/messages
  // QoS depth 10: only recent messages matter.
  status_mode_pub_ = this->create_publisher<std_msgs::msg::String>("/behavior/status", 10);
  comms_pub_ = this->create_publisher<std_msgs::msg::String>("/behavior/comms", 10);

  // TRAVERSE GOAL SUBSCRIPTION ----------------------------------------------------
  // High‑level navigation targets produced by UI or autonomy planners.
  // Using a dedicated behavior namespace keeps ownership explicit.
  traverse_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/behavior/traverse_goal",
      10,
      std::bind(&BehaviorController::traverseGoalCallback, this, std::placeholders::_1));

  // RETURN-TO-BASE SUBSCRIPTION ---------------------------------------------------
  // Semantically distinct from a generic traverse even though the payload is the same.
  // Allows differentiated logging / future logic (e.g. validating dock pose).
  rtb_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/behavior/return_to_base",
    10,
    std::bind(&BehaviorController::returnBaseCallback, this, std::placeholders::_1));

  // MANUAL MODE SUBSCRIPTIONS -----------------------------------------------------
  //  /behavior/manual_enable : std_msgs/Bool toggling MANUAL mode
  //  /behavior/manual_cmd    : geometry_msgs/Twist forwarded to /cmd_vel (only if MANUAL)
  // This keeps manual teleop outside Nav2, letting you override quickly.
  manual_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/behavior/manual_enable", 10,
    std::bind(&BehaviorController::manualEnableCallback, this, std::placeholders::_1));
  manual_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/behavior/manual_cmd", 10,
    std::bind(&BehaviorController::manualCmdCallback, this, std::placeholders::_1));

  // INSPECTION LENGTH SUBSCRIPTION ----------------------------------------------
  inspect_len_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/behavior/inspect_length", 10,
    std::bind(&BehaviorController::inspectLengthCallback, this, std::placeholders::_1));

  // MANUAL VELOCITY OUTPUT --------------------------------------------------------
  // Single publisher used both to: (a) forward manual commands, (b) send zero on mode changes
  // or E‑STOP. Centralizing this makes future velocity smoothing easier.
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // E-STOP SUBSCRIPTION -----------------------------------------------------------
  // Momentary semantics here: only a TRUE pulse matters. We ignore FALSE so the source
  // (e.g. a button) doesn't need to send a reset. To implement latching, store a flag.
  estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/estop",
      10,
      std::bind(&BehaviorController::estopCallback, this, std::placeholders::_1));

  // NAV2 ACTION CLIENT ------------------------------------------------------------
  // We intentionally use the interface-based factory because shared_from_this() is unsafe
  // within the constructor (object not yet owned by a shared_ptr). If you later move
  // initialization to on_activate() or a separate init() method you could use shared_from_this.
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "navigate_to_pose"); // Standard Nav2 action name.

  // Waypoints action client
  wps_client_ = rclcpp_action::create_client<FollowWaypoints>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "follow_waypoints");

  // INITIAL STATUS ----------------------------------------------------------------
  // Helps UI show a non-empty label and confirms node startup.
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
    case RobotMode::ESTOPPED:    return "ESTOPPED"; // retained for future if needed
  }
  return "UNKNOWN";
}

/**
 * @brief Handle an Emergency Stop input (momentary semantics).
 *
 * Behavior:
 *  - Only reacts when msg->data == true (a pulse)
 *  - Cancels any active Nav2 goal
 *  - If we were in TRAVERSING mode we fall back to IDLE
 *  - Publishes a zero Twist to bring the robot to a halt
 *  - DOES NOT set a persistent latching state (new goals allowed immediately)
 *
 * To implement a latching E‑STOP:
 *  (1) Add a bool estop_latched_ member
 *  (2) Set estop_latched_ true here
 *  (3) Gate traverseGoalCallback() & manualEnableCallback()
 *  (4) Add a service or a FALSE message path to clear it
 */
void BehaviorController::estopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg || !msg->data) return; // only act when true
  if (active_goal_) {
    nav_client_->async_cancel_goal(active_goal_);
    active_goal_.reset();
  }
  if (mode_ == RobotMode::TRAVERSING) {
    mode_ = RobotMode::IDLE; // drop to idle from autonomous nav
  }
  publishZeroTwist();
  clearInspection("E-STOP");
  publishComms("E-STOP pressed: goals canceled, motion halted");
  publishMode();
}

/**
 * @brief Callback for generic navigation ("traverse") goals.
 *
 * Accepts any incoming PoseStamped and forwards it to Nav2 via sendTraverseGoal().
 * We intentionally do **no validation** here (map bounds, obstacles, etc.) – that
 * responsibility typically lives in Nav2 or an upstream planner. Add checks here if
 * you want to reject obviously invalid frames / poses.
 */
void BehaviorController::traverseGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Accept new traverse goal regardless of prior e-stop (momentary model)
  if (!msg) return;
  // If manual control is active, do not accept autonomous traverse goals
  if (mode_ == RobotMode::MANUAL) {
    publishComms("Traverse goal ignored -> MANUAL");
    return;
  }
  publishComms("Received traverse goal: (" +
                std::to_string(msg->pose.position.x) + ", " +
                std::to_string(msg->pose.position.y) + ")");
  // Remember center for potential inspection
  last_traverse_goal_ = *msg;
  sendTraverseGoal(*msg);
}

/**
 * @brief Callback for Return-To-Base (RTB) goals.
 *
 * Currently identical to traverseGoalCallback except for log prefix. Keeping a
 * separate topic + callback makes it trivial to layer special logic (e.g., verify
 * the base pose is known, enforce docking orientation, etc.).
 */
void BehaviorController::returnBaseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!msg) return;
  // If manual control is active, do not accept RTB goals
  if (mode_ == RobotMode::MANUAL) {
    publishComms("RTB goal ignored -> MANUAL");
    return;
  }
  publishComms("Received RTB goal: (" + std::to_string(msg->pose.position.x) + ", " +
                std::to_string(msg->pose.position.y) + ")");
  sendTraverseGoal(*msg);
}

/**
 * @brief Enable or disable MANUAL velocity override mode.
 *
 * When enabling MANUAL:
 *  - Any active autonomous navigation goal is canceled (manual takes priority)
 *  - Mode transitions to MANUAL
 *  - A zero Twist is published so the operator starts from rest
 *
 * When disabling MANUAL:
 *  - A zero Twist is published (good hygiene; stops drift)
 *  - Mode returns to IDLE
 *
 * NOTE: We do not attempt to restore a previously canceled traverse goal. That would
 * require storing a copy of the last goal and perhaps a policy about resumption.
 */
void BehaviorController::manualEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    if (mode_ != RobotMode::MANUAL) {
      if (active_goal_) { // Cancel any autonomous navigation
        nav_client_->async_cancel_goal(active_goal_);
        active_goal_.reset();
      }
      mode_ = RobotMode::MANUAL;
      publishComms("Manual mode enabled");
      publishZeroTwist(); // start from rest
      clearInspection("Manual enabled");
      publishMode();
    }
  } else {
    if (mode_ == RobotMode::MANUAL) {
      publishZeroTwist();
      mode_ = RobotMode::IDLE;
      publishComms("Manual mode disabled");
      publishMode();
      // Do not auto-resume inspection; require new input
    }
  }
}

/**
 * @brief Forward operator velocity commands while in MANUAL mode.
 *
 * If you need safety constraints (max speed, acceleration smoothing, deadman switch),
 * this is the interception point. For example, clamp msg->linear.x before publishing.
 */
void BehaviorController::manualCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (mode_ != RobotMode::MANUAL) return;
  if (cmd_vel_pub_) cmd_vel_pub_->publish(*msg);
}

/**
 * @brief Publish a zeroed Twist (stop motion) if publisher exists.
 *
 * Central helper used by E‑STOP, mode transitions, and could be reused by a future
 * watchdog timer. Abstracting it avoids duplicate boilerplate and keeps semantics clear.
 */
void BehaviorController::publishZeroTwist() {
  if (!cmd_vel_pub_) return;
  geometry_msgs::msg::Twist z; cmd_vel_pub_->publish(z);
}

//
// Internal: Forward a goal to Nav2 NavigateToPose action.
//  - Wait briefly for server.
//  - Populate goal.
//  - Set up callbacks for acceptance and result.
//  - Does not (yet) provide feedback callback; can be added later.
//
/**
 * @brief Forward a PoseStamped to Nav2's NavigateToPose action asynchronously.
 *
 * Flow:
 *  1. Optionally wait for the action server (short timeout here for responsiveness)
 *  2. Populate NavigateToPose::Goal with the pose (header frame + pose forwarded verbatim)
 *  3. Register callbacks for goal acceptance and result
 *  4. Send goal; on acceptance we store a GoalHandle so we can later cancel
 *
 * Notes & Extension Points:
 *  - Server Wait: Currently 500ms. For more robustness, implement a retry loop or
 *    declare a parameter controlling this timeout.
 *  - Feedback: Add opts.feedback_callback to stream progress (e.g., distance remaining).
 *  - Goal Replacement Policy: Right now we allow a new goal to be sent even if one is
 *    active (Nav2 will reject or preempt depending on configuration). You could enforce
 *    a single-goal-at-a-time policy by canceling first or rejecting new ones while
 *    active_goal_ is non-null.
 *  - Frame Handling: Assumes caller provides a frame Nav2 understands (e.g. map). If you
 *    want to accept robot-relative goals, transform them here using TF2.
 */
void BehaviorController::sendTraverseGoal(const geometry_msgs::msg::PoseStamped &pose) {
  // SHORT SERVER AVAILABILITY CHECK ------------------------------------------------
  // In production you might:
  //  - Retry longer with exponential backoff
  //  - Publish a special degraded status
  //  - Trigger a recovery behavior
  if (!nav_client_->wait_for_action_server(500ms)) {
    publishComms("Nav2 action server not available");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = pose; // Copy entire PoseStamped (header + pose).

  // Configure asynchronous goal send options.
  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // GOAL RESPONSE CALLBACK --------------------------------------------------------
  // Triggered exactly once: either accepted (handle valid) or rejected (null handle).
  opts.goal_response_callback = [this](std::shared_ptr<GoalHandleNav> handle) {
    if (!handle) {
      publishComms("Traverse goal rejected by server");
    } else {
      active_goal_ = handle;
      // If we're in a patrol sequence, keep high-level mode as PATROLLING
      if (inspecting_) {
        mode_ = RobotMode::PATROLLING;
      } else {
        mode_ = RobotMode::TRAVERSING;
      }
      publishComms("Traverse goal accepted");
      publishMode();
    }
  };

  // RESULT CALLBACK ---------------------------------------------------------------
  // Fires on terminal state. We always transition back to IDLE (simple policy).
  // For richer behavior you could inspect outcome and branch (e.g., if aborted due to
  // planning failure, attempt a smaller goal, etc.).
  opts.result_callback = [this](const GoalHandleNav::WrappedResult &res) {
    std::string outcome;
    switch (res.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: outcome = "SUCCEEDED"; break;
      case rclcpp_action::ResultCode::ABORTED:   outcome = "ABORTED";   break;
      case rclcpp_action::ResultCode::CANCELED:  outcome = "CANCELED";  break;
      default:                                   outcome = "UNKNOWN";   break;
    }
    active_goal_.reset();
    // After a traverse completes, return to IDLE; patrol (if any) will update mode on accept
    mode_ = RobotMode::IDLE;
    publishComms("Traverse result: " + outcome);
    publishMode();

    // On success, if an inspection was requested and not yet started, start it.
    if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
      startInspectionIfRequested();
    }
  };

  // (Future) FEEDBACK CALLBACK ----------------------------------------------------
  // Example skeleton if you add it:
  // opts.feedback_callback = [this](GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> & fb){
  //   publishStatus("Remaining distance: " + std::to_string(fb->distance_remaining));
  // };

  nav_client_->async_send_goal(goal, opts);
}







// ---------------- INSPECTION FLOW ----------------------------------------------
void BehaviorController::inspectLengthCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  if (!msg) return;
  pending_inspect_L_ = std::max(0.0, msg->data);
  if (pending_inspect_L_ > 0.0) {
    publishComms("Inspection requested: L=" + std::to_string(pending_inspect_L_) + " m");
  }
}

void BehaviorController::startInspectionIfRequested() {
  if (pending_inspect_L_ <= 0.0) return;
  // Generate waypoints for an LxL square centered at last traverse goal
  auto waypoints = generate_patrol_waypoints(last_traverse_goal_, pending_inspect_L_);
  // Consume the request regardless of outcome
  pending_inspect_L_ = 0.0;
  if (waypoints.empty()) {
    publishComms("Inspection requested but no waypoints generated");
    return;
  }
  publishComms("Patrolling in progress. Waypoints=" + std::to_string(waypoints.size()));
  sendWaypointList(waypoints);
}

// (sequential fallback removed for simplicity; FollowWaypoints handles the sequence)

void BehaviorController::sendWaypointList(const std::vector<geometry_msgs::msg::PoseStamped> &poses) {
  if (!wps_client_ || !wps_client_->wait_for_action_server(500ms)) {
    publishComms("FollowWaypoints server not available; cannot start patrolling");
    return;
  }

  FollowWaypoints::Goal goal;
  goal.poses = poses;

  auto opts = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  opts.goal_response_callback = [this](std::shared_ptr<GoalHandleWps> handle){
    if (!handle) {
      publishComms("Patrolling goal rejected");
    } else {
      mode_ = RobotMode::PATROLLING;
      publishComms("Patrolling in progress");
      publishMode();
    }
  };
  opts.feedback_callback = [this](GoalHandleWps::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> & fb){
    (void)fb; // Optionally report progress
  };
  opts.result_callback = [this](const GoalHandleWps::WrappedResult &res){
    std::string outcome;
    switch (res.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: outcome = "SUCCEEDED"; break;
      case rclcpp_action::ResultCode::ABORTED: outcome = "ABORTED"; break;
      case rclcpp_action::ResultCode::CANCELED: outcome = "CANCELED"; break;
      default: outcome = "UNKNOWN"; break;
    }
    // Patrol complete/terminated; return to IDLE for next command
    mode_ = RobotMode::IDLE;
    publishComms("Patrolling result: " + outcome);
    publishMode();
    clearInspection("Patrolling done");

    // If patrol succeeded, send a final traverse goal to the base at (0,0)
    if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
      publishComms("Returning to base (0,0)");
      geometry_msgs::msg::PoseStamped base_pose;
      base_pose.header.frame_id = "map";
      base_pose.header.stamp = this->now();
      base_pose.pose.position.x = 0.0;
      base_pose.pose.position.y = 0.0;
      base_pose.pose.orientation.z = 0.0;
      base_pose.pose.orientation.w = 1.0;
      // sendTraverseGoal will set mode and publish mode on accept
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