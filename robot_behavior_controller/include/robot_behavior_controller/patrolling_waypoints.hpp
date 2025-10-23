#pragma once

#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace robot_behavior_controller {

// Generate a sequence of waypoints to inspect an LxL square centered at `center`.
// Pattern selection by L (meters):
//  - small (<= 5 m): middle cross (horizontal then vertical through center)
//  - medium (<= 15 m): diagonals (both main and anti-diagonals)
//  - large (> 15 m): lawnmower pattern with ~2 m stripe spacing
std::vector<geometry_msgs::msg::PoseStamped>
generate_patrol_waypoints(const geometry_msgs::msg::PoseStamped &center, double L);

} // namespace robot_behavior_controller

