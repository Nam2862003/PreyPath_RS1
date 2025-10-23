#include "robot_behavior_controller/patrolling_waypoints.hpp"
#include <cmath>

namespace robot_behavior_controller {

static geometry_msgs::msg::PoseStamped mkpose(double x, double y, const geometry_msgs::msg::PoseStamped &center){
  geometry_msgs::msg::PoseStamped p = center;
  p.pose.position.x = x;
  p.pose.position.y = y;
  // Keep heading as center's orientation (simple policy)
  return p;
}

std::vector<geometry_msgs::msg::PoseStamped>
generate_patrol_waypoints(const geometry_msgs::msg::PoseStamped &center, double L){
  std::vector<geometry_msgs::msg::PoseStamped> pts;
  const double cx = center.pose.position.x;
  const double cy = center.pose.position.y;
  const double half = 0.5 * std::max(0.0, L);

  if (L <= 5.0) {
    // Small: patroling perimeter of the square
    pts.push_back(mkpose(cx - half, cy - half, center));
    pts.push_back(mkpose(cx - half, cy + half, center));
    pts.push_back(mkpose(cx + half, cy + half, center));
    pts.push_back(mkpose(cx + half, cy - half, center));
  } else if (L <= 15.0) {
    // Medium: patroling diagonally throught the middles of the perimeter
    pts.push_back(mkpose(cx - half, cy, center));
    pts.push_back(mkpose(cx, cy - half, center));
    pts.push_back(mkpose(cx + half, cy, center));
    pts.push_back(mkpose(cx, cy + half, center));
  } else {
    // Large: lawnmower with ~5m stripe spacing, start at top-left
    const double stripe = 5.0;
    const int n = std::max(1, static_cast<int>(std::ceil(L / stripe)));
    const double step = (n > 1) ? (L / (n - 1)) : 0.0;
    // We'll sweep from cy+half down to cy-half, alternating direction in x
    bool left_to_right = true;
    for (int i = 0; i < n; ++i) {
      const double y = cy + half - i * step;
      if (left_to_right) {
        pts.push_back(mkpose(cx - half, y, center));
        pts.push_back(mkpose(cx + half, y, center));
      } else {
        pts.push_back(mkpose(cx + half, y, center));
        pts.push_back(mkpose(cx - half, y, center));
      }
      left_to_right = !left_to_right;
    }
  }
  return pts;
}

} // namespace robot_behavior_controller
