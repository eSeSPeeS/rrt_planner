#include "rrt_planner/rrt_planner.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav2_core::GlobalPlanner)

namespace rrt_planner
{

// ──────────────────────────────────────────────────────────────────────────────
// Helper – yaw → quaternion
// ──────────────────────────────────────────────────────────────────────────────

geometry_msgs::msg::Quaternion RRTPlanner::yawToQuat(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

// ──────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ──────────────────────────────────────────────────────────────────────────────

void RRTPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_         = parent;
  name_         = name;
  tf_           = tf;
  costmap_ros_  = costmap_ros;
  costmap_      = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  auto node = node_.lock();
  if (!node) { throw std::runtime_error("RRTPlanner: failed to lock parent node"); }

  // ── RRT parameters ────────────────────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations",       rclcpp::ParameterValue(5000));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".step_size",            rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_tolerance",       rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_bias",            rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".collision_check_step", rclcpp::ParameterValue(0.02));

  node->get_parameter(name_ + ".max_iterations",       max_iterations_);
  node->get_parameter(name_ + ".step_size",            step_size_);
  node->get_parameter(name_ + ".goal_tolerance",       goal_tolerance_);
  node->get_parameter(name_ + ".goal_bias",            goal_bias_);
  node->get_parameter(name_ + ".collision_check_step", collision_check_step_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_margin", rclcpp::ParameterValue(0.0));
  node->get_parameter(name_ + ".safety_margin", safety_margin_);
  safety_margin_ = std::clamp(safety_margin_, 0.0, 1.0);

  // ── Visualisation parameters ──────────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".node_scale", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".edge_scale", rclcpp::ParameterValue(0.01));

  node->get_parameter(name_ + ".node_scale", node_scale_);
  node->get_parameter(name_ + ".edge_scale", edge_scale_);

  // ── Publisher (transient-local = latched) ─────────────────────────────────
  rclcpp::QoS qos(1);
  qos.transient_local();
  tree_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    name_ + "/rrt_tree", qos);

  // ── RNG ───────────────────────────────────────────────────────────────────
  rng_.seed(std::chrono::steady_clock::now().time_since_epoch().count());
  uniform01_ = std::uniform_real_distribution<double>(0.0, 1.0);

  last_goal_x_ = std::numeric_limits<double>::quiet_NaN();
  last_goal_y_ = std::numeric_limits<double>::quiet_NaN();

  RCLCPP_INFO(node->get_logger(),
    "RRTPlanner v6 configured: max_iter=%d  step=%.3f m  tol=%.3f m  bias=%.2f  "
    "safety=%.2f  tree topic: %s/rrt_tree",
    max_iterations_, step_size_, goal_tolerance_, goal_bias_, safety_margin_, name_.c_str());
}

void RRTPlanner::cleanup()
{
  tree_.clear();
  tree_pub_.reset();
  last_goal_x_ = std::numeric_limits<double>::quiet_NaN();
  last_goal_y_ = std::numeric_limits<double>::quiet_NaN();
  RCLCPP_INFO(node_.lock()->get_logger(), "RRTPlanner cleaned up.");
}

void RRTPlanner::activate()
{
  tree_pub_->on_activate();
  RCLCPP_INFO(node_.lock()->get_logger(), "RRTPlanner activated.");
}

void RRTPlanner::deactivate()
{
  tree_pub_->on_deactivate();
  RCLCPP_INFO(node_.lock()->get_logger(), "RRTPlanner deactivated.");
}

// ──────────────────────────────────────────────────────────────────────────────
// createPlan
// ──────────────────────────────────────────────────────────────────────────────

nav_msgs::msg::Path RRTPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto node = node_.lock();
  if (!node) { throw std::runtime_error("RRTPlanner: node expired"); }

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp    = node->now();

  const double sx = start.pose.position.x;
  const double sy = start.pose.position.y;
  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;

  // ── Reset tree on new goal ────────────────────────────────────────────────
  if (goalChanged(gx, gy)) {
    RCLCPP_INFO(node->get_logger(),
      "RRTPlanner: new goal (%.2f, %.2f) – rebuilding tree from (%.2f, %.2f)",
      gx, gy, sx, sy);
    clearTreeMarkers(node->now());
    tree_.clear();
    tree_.emplace_back(sx, sy, -1);   // V ← {x_init}; E ← ∅
    last_goal_x_ = gx;
    last_goal_y_ = gy;
  } else {
    RCLCPP_DEBUG(node->get_logger(),
      "RRTPlanner: same goal – continuing with existing tree (%zu nodes)",
      tree_.size());
  }

  // ── Check if goal already in tree ────────────────────────────────────────
  int goal_node_idx = -1;
  for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
    if (goalReached(tree_[i].x, tree_[i].y, gx, gy)) {
      goal_node_idx = i;
      break;
    }
  }

  // ── Algorithm 3: RRT ─────────────────────────────────────────────────────
  if (goal_node_idx == -1) {
    for (int i = 0; i < max_iterations_; ++i) {
      auto [rx, ry]     = sampleFree(gx, gy);
      int  nearest_idx  = nearest(rx, ry);
      const RRTNode & x_nearest = tree_[nearest_idx];
      auto [nx, ny]     = steer(x_nearest.x, x_nearest.y, rx, ry);

      if (obstacleFree(x_nearest.x, x_nearest.y, nx, ny)) {
        tree_.emplace_back(nx, ny, nearest_idx);
        int new_idx = static_cast<int>(tree_.size()) - 1;

        if (goalReached(nx, ny, gx, gy)) {
          goal_node_idx = new_idx;
          RCLCPP_INFO(node->get_logger(),
            "RRTPlanner: goal reached after %d iterations (%zu nodes)",
            i + 1, tree_.size());
          break;
        }
      }
    }
  }

  // ── Publish full tree to RViz ─────────────────────────────────────────────
  publishTree(node->now());

  if (goal_node_idx == -1) {
    RCLCPP_WARN(node->get_logger(),
      "RRTPlanner: goal not reached within %d iterations "
      "(tree has %zu nodes – will continue next call).",
      max_iterations_, tree_.size());
    return path;
  }

  path = tracePath(goal_node_idx, gx, gy, goal, global_frame_, node->now());
  return path;
}

// ──────────────────────────────────────────────────────────────────────────────
// tracePath  –  KEY FIX: each pose oriented toward the NEXT waypoint
// ──────────────────────────────────────────────────────────────────────────────

nav_msgs::msg::Path RRTPlanner::tracePath(
  int goal_node_idx,
  double /*goal_x*/, double /*goal_y*/,
  const geometry_msgs::msg::PoseStamped & goal_pose_stamped,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp    = stamp;

  // Walk parent pointers from goal → root, then reverse to get root → goal
  std::vector<int> indices;
  for (int idx = goal_node_idx; idx != -1; idx = tree_[idx].parent_idx) {
    indices.push_back(idx);
  }
  std::reverse(indices.begin(), indices.end());

  path.poses.reserve(indices.size() + 1);

  for (size_t i = 0; i < indices.size(); ++i) {
    const RRTNode & cur = tree_[indices[i]];

    // Determine where this pose is "looking":
    //   - for all but the last tree node  → look at the next tree node
    //   - for the last tree node          → look at the exact goal pose
    double next_x, next_y;
    if (i + 1 < indices.size()) {
      const RRTNode & nxt = tree_[indices[i + 1]];
      next_x = nxt.x;
      next_y = nxt.y;
    } else {
      next_x = goal_pose_stamped.pose.position.x;
      next_y = goal_pose_stamped.pose.position.y;
    }

    double yaw = std::atan2(next_y - cur.y, next_x - cur.x);

    geometry_msgs::msg::PoseStamped ps;
    ps.header          = path.header;
    ps.pose.position.x = cur.x;
    ps.pose.position.y = cur.y;
    ps.pose.position.z = 0.0;
    ps.pose.orientation = yawToQuat(yaw);
    path.poses.push_back(ps);
  }

  // Append the exact goal pose with the goal's own orientation
  geometry_msgs::msg::PoseStamped goal_ps = goal_pose_stamped;
  goal_ps.header = path.header;
  path.poses.push_back(goal_ps);

  return path;
}

// ──────────────────────────────────────────────────────────────────────────────
// publishTree
// ──────────────────────────────────────────────────────────────────────────────

void RRTPlanner::publishTree(const rclcpp::Time & stamp) const
{
  if (!tree_pub_ || !tree_pub_->is_activated() || tree_.empty()) { return; }

  visualization_msgs::msg::MarkerArray ma;

  // Marker 0: nodes (SPHERE_LIST)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp    = stamp;
    m.ns              = "rrt_nodes";
    m.id              = 0;
    m.type            = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action          = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = node_scale_;
    m.color.r = node_r_; m.color.g = node_g_;
    m.color.b = node_b_; m.color.a = node_a_;
    m.pose.orientation.w = 1.0;

    m.points.reserve(tree_.size());
    for (const auto & n : tree_) {
      geometry_msgs::msg::Point p;
      p.x = n.x; p.y = n.y; p.z = 0.02;
      m.points.push_back(p);
    }
    ma.markers.push_back(m);
  }

  // Marker 1: edges (LINE_LIST)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp    = stamp;
    m.ns              = "rrt_edges";
    m.id              = 1;
    m.type            = visualization_msgs::msg::Marker::LINE_LIST;
    m.action          = visualization_msgs::msg::Marker::ADD;
    m.scale.x         = edge_scale_;
    m.color.r = edge_r_; m.color.g = edge_g_;
    m.color.b = edge_b_; m.color.a = edge_a_;
    m.pose.orientation.w = 1.0;

    m.points.reserve(2 * tree_.size());
    for (int i = 1; i < static_cast<int>(tree_.size()); ++i) {
      const RRTNode & child  = tree_[i];
      const RRTNode & parent = tree_[child.parent_idx];

      geometry_msgs::msg::Point pc, pp;
      pp.x = parent.x; pp.y = parent.y; pp.z = 0.02;
      pc.x = child.x;  pc.y = child.y;  pc.z = 0.02;
      m.points.push_back(pp);
      m.points.push_back(pc);
    }
    ma.markers.push_back(m);
  }

  tree_pub_->publish(ma);
}

// ──────────────────────────────────────────────────────────────────────────────
// clearTreeMarkers
// ──────────────────────────────────────────────────────────────────────────────

void RRTPlanner::clearTreeMarkers(const rclcpp::Time & stamp) const
{
  if (!tree_pub_ || !tree_pub_->is_activated()) { return; }

  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker del;
  del.header.frame_id = global_frame_;
  del.header.stamp    = stamp;
  del.action          = visualization_msgs::msg::Marker::DELETEALL;
  ma.markers.push_back(del);
  tree_pub_->publish(ma);
}

// ──────────────────────────────────────────────────────────────────────────────
// SampleFree
// ──────────────────────────────────────────────────────────────────────────────

std::pair<double, double> RRTPlanner::sampleFree(
  double goal_x, double goal_y)
{
  if (uniform01_(rng_) < goal_bias_) { return {goal_x, goal_y}; }

  const double ox = costmap_->getOriginX();
  const double oy = costmap_->getOriginY();
  std::uniform_real_distribution<double> dx(ox, ox + costmap_->getSizeInMetersX());
  std::uniform_real_distribution<double> dy(oy, oy + costmap_->getSizeInMetersY());

  for (int attempt = 0; attempt < 100; ++attempt) {
    double x = dx(rng_), y = dy(rng_);
    unsigned int mx, my;
    if (!worldToMap(x, y, mx, my)) { continue; }
    if (costmap_->getCost(mx, my) < nav2_costmap_2d::LETHAL_OBSTACLE) {
      return {x, y};
    }
  }
  return {goal_x, goal_y};
}

// ──────────────────────────────────────────────────────────────────────────────
// Nearest
// ──────────────────────────────────────────────────────────────────────────────

int RRTPlanner::nearest(double x, double y) const
{
  int    best_idx  = 0;
  double best_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
    double d = (tree_[i].x - x) * (tree_[i].x - x) +
               (tree_[i].y - y) * (tree_[i].y - y);
    if (d < best_dist) { best_dist = d; best_idx = i; }
  }
  return best_idx;
}

// ──────────────────────────────────────────────────────────────────────────────
// Steer
// ──────────────────────────────────────────────────────────────────────────────

std::pair<double, double> RRTPlanner::steer(
  double from_x, double from_y, double to_x, double to_y) const
{
  double dx = to_x - from_x, dy = to_y - from_y;
  double dist = std::hypot(dx, dy);
  if (dist <= step_size_) { return {to_x, to_y}; }
  double s = step_size_ / dist;
  return {from_x + dx * s, from_y + dy * s};
}

// ──────────────────────────────────────────────────────────────────────────────
// ObstacleFree  –  respects inflation zones
// ──────────────────────────────────────────────────────────────────────────────

bool RRTPlanner::obstacleFree(
  double x0, double y0, double x1, double y1) const
{
  double dx = x1 - x0, dy = y1 - y0;
  double dist = std::hypot(dx, dy);
  int steps = std::max(1, static_cast<int>(std::ceil(dist / collision_check_step_)));

  for (int s = 0; s <= steps; ++s) {
    double t = static_cast<double>(s) / steps;
    unsigned int mx, my;
    if (!worldToMap(x0 + t * dx, y0 + t * dy, mx, my)) { return false; }
    // threshold: 253*(1-0) = 253 (brak marginesu) → 253*(1-1) = 0 (max margines)
    const unsigned char threshold = static_cast<unsigned char>(
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE * (1.0 - safety_margin_));
    if (costmap_->getCost(mx, my) >= threshold) {
      return false;
    }
  }
  return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Helpers
// ──────────────────────────────────────────────────────────────────────────────

bool RRTPlanner::goalReached(double x, double y, double gx, double gy) const
{
  return std::hypot(x - gx, y - gy) <= goal_tolerance_;
}

bool RRTPlanner::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

bool RRTPlanner::goalChanged(double gx, double gy) const
{
  if (std::isnan(last_goal_x_) || std::isnan(last_goal_y_)) { return true; }
  return std::hypot(gx - last_goal_x_, gy - last_goal_y_) > goal_tolerance_;
}

}  // namespace rrt_planner