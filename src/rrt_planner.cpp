#include "rrt_planner/rrt_planner.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <thread>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav2_core::GlobalPlanner)

namespace rrt_planner
{

geometry_msgs::msg::Quaternion RRTPlanner::yawToQuat(double yaw)
{
  tf2::Quaternion q; q.setRPY(0.0, 0.0, yaw); q.normalize();
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x(); msg.y = q.y(); msg.z = q.z(); msg.w = q.w();
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
  node_ = parent; name_ = name; tf_ = tf;
  costmap_ros_  = costmap_ros;
  costmap_      = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  auto node = node_.lock();
  if (!node) { throw std::runtime_error("RRTPlanner: failed to lock parent node"); }

  nav2_util::declare_parameter_if_not_declared(node, name_+".max_iterations",       rclcpp::ParameterValue(5000));
  nav2_util::declare_parameter_if_not_declared(node, name_+".step_size",            rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(node, name_+".goal_tolerance",       rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, name_+".goal_bias",            rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(node, name_+".collision_check_step", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(node, name_+".gamma_rrt_star",       rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(node, name_+".post_goal_iterations", rclcpp::ParameterValue(500));
  nav2_util::declare_parameter_if_not_declared(node, name_+".node_scale",           rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(node, name_+".edge_scale",           rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(node, name_+".viz_publish_every_n",  rclcpp::ParameterValue(10));
  nav2_util::declare_parameter_if_not_declared(node, name_+".viz_step_delay_ms",    rclcpp::ParameterValue(20));

  node->get_parameter(name_+".max_iterations",       max_iterations_);
  node->get_parameter(name_+".step_size",            step_size_);
  node->get_parameter(name_+".goal_tolerance",       goal_tolerance_);
  node->get_parameter(name_+".goal_bias",            goal_bias_);
  node->get_parameter(name_+".collision_check_step", collision_check_step_);
  node->get_parameter(name_+".gamma_rrt_star",       gamma_rrt_star_);
  nav2_util::declare_parameter_if_not_declared(node, name_+".max_near_radius", rclcpp::ParameterValue(0.40));
  node->get_parameter(name_+".max_near_radius", max_near_radius_);
  node->get_parameter(name_+".post_goal_iterations", post_goal_iterations_);
  node->get_parameter(name_+".node_scale",           node_scale_);
  node->get_parameter(name_+".edge_scale",           edge_scale_);
  node->get_parameter(name_+".viz_publish_every_n",  viz_publish_every_n_);
  node->get_parameter(name_+".viz_step_delay_ms",    viz_step_delay_ms_);

  rclcpp::QoS qos(1); qos.transient_local();
  tree_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    name_ + "/rrt_tree", qos);

  rng_.seed(std::chrono::steady_clock::now().time_since_epoch().count());
  uniform01_ = std::uniform_real_distribution<double>(0.0, 1.0);

  last_goal_x_ = std::numeric_limits<double>::quiet_NaN();
  last_goal_y_ = std::numeric_limits<double>::quiet_NaN();

  RCLCPP_INFO(node->get_logger(),
    "RRTPlanner v9 (RRT*): max_iter=%d  step=%.3f m  tol=%.3f m  "
    "bias=%.2f  gamma=%.2f  post_goal_iter=%d  viz_every=%d  delay=%d ms",
    max_iterations_, step_size_, goal_tolerance_, goal_bias_,
    gamma_rrt_star_, post_goal_iterations_,
    viz_publish_every_n_, viz_step_delay_ms_);
}

void RRTPlanner::cleanup()
{
  tree_.clear(); tree_pub_.reset();
  planning_done_ = false; best_goal_node_idx_ = -1;
  best_goal_cost_ = std::numeric_limits<double>::max();
  last_goal_x_ = last_goal_y_ = std::numeric_limits<double>::quiet_NaN();
  RCLCPP_INFO(node_.lock()->get_logger(), "RRTPlanner (RRT*) cleaned up.");
}

void RRTPlanner::activate()
{
  tree_pub_->on_activate();
  RCLCPP_INFO(node_.lock()->get_logger(), "RRTPlanner (RRT*) activated.");
}

void RRTPlanner::deactivate()
{
  tree_pub_->on_deactivate();
  RCLCPP_INFO(node_.lock()->get_logger(), "RRTPlanner (RRT*) deactivated.");
}

// ──────────────────────────────────────────────────────────────────────────────
// createPlan  –  RRT* z fazą post-goal rewiring i zamrożeniem planu
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

  const double sx = start.pose.position.x, sy = start.pose.position.y;
  const double gx = goal.pose.position.x,  gy = goal.pose.position.y;

  // ── Nowy cel → reset ──────────────────────────────────────────────────────
  if (goalChanged(gx, gy)) {
    RCLCPP_INFO(node->get_logger(),
      "RRTPlanner*: new goal (%.2f,%.2f) – resetting from (%.2f,%.2f)",
      gx, gy, sx, sy);
    clearTreeMarkers(node->now());
    tree_.clear();
    tree_.emplace_back(sx, sy, -1, 0.0);
    last_goal_x_ = gx; last_goal_y_ = gy;
    planning_done_      = false;
    best_goal_node_idx_ = -1;
    best_goal_cost_     = std::numeric_limits<double>::max();
  }

  // ── Plan zamrożony – robot jedzie – zwróć cached ścieżkę ─────────────────
  if (planning_done_ && best_goal_node_idx_ >= 0 &&
      best_goal_node_idx_ < static_cast<int>(tree_.size()))
  {
    RCLCPP_DEBUG(node->get_logger(),
      "RRTPlanner*: plan frozen – returning cached path (cost=%.3f)", best_goal_cost_);
    return tracePath(best_goal_node_idx_, goal, global_frame_, node->now());
  }

  // ── Algorithm 6: RRT* ────────────────────────────────────────────────────
  int nodes_since_pub = 0;
  int post_goal_count = 0;

  for (int i = 0; i < max_iterations_; ++i) {

    // Faza 2: po znalezieniu celu – odliczaj post_goal_iterations_
    if (best_goal_node_idx_ != -1) {
      ++post_goal_count;
      if (post_goal_count > post_goal_iterations_) {
        RCLCPP_INFO(node->get_logger(),
          "RRTPlanner*: rewiring done (%d iter, cost=%.3f, %zu nodes) – FREEZING",
          post_goal_count - 1, best_goal_cost_, tree_.size());
        planning_done_ = true;
        break;
      }
    }

    // Linia 3-5: SampleFree → Nearest → Steer
    auto [rx, ry]   = sampleFree(gx, gy);
    int  near_id    = nearest(rx, ry);
    auto [nx, ny]   = steer(tree_[near_id].x, tree_[near_id].y, rx, ry);

    // Linia 6: ObstacleFree
    if (!obstacleFree(tree_[near_id].x, tree_[near_id].y, nx, ny)) { continue; }

    // Linia 7: Near
    double r = nearRadius();
    std::vector<int> X_near = near(nx, ny, r);

    // Linie 9-12: Choose Parent
    int    x_min = near_id;
    double c_min = tree_[near_id].cost + std::hypot(nx-tree_[near_id].x, ny-tree_[near_id].y);
    for (int ni : X_near) {
      double c = tree_[ni].cost + std::hypot(nx-tree_[ni].x, ny-tree_[ni].y);
      if (c < c_min && obstacleFree(tree_[ni].x, tree_[ni].y, nx, ny)) {
        x_min = ni; c_min = c;
      }
    }

    // Linia 8+13: dodaj węzeł
    tree_.emplace_back(nx, ny, x_min, c_min);
    int new_idx = static_cast<int>(tree_.size()) - 1;
    ++nodes_since_pub;

    // Linie 14-16: Rewire
    for (int ni : X_near) {
      double c = tree_[new_idx].cost + std::hypot(tree_[ni].x-nx, tree_[ni].y-ny);
      if (c < tree_[ni].cost && obstacleFree(nx, ny, tree_[ni].x, tree_[ni].y)) {
        tree_[ni].parent_idx = new_idx;
        tree_[ni].cost       = c;
        // Jeśli to węzeł celu – zaktualizuj best_goal_cost_
        if (ni == best_goal_node_idx_) { best_goal_cost_ = c; }
        // POPRAWKA: propaguj zaktualizowany koszt do wszystkich potomków,
        // w tym do węzła celu jeśli jest gdzieś głębiej w drzewie
        propagateCosts(ni);
      }
    }

    // Sprawdź czy osiągnęliśmy cel / poprawiliśmy koszt
    if (goalReached(nx, ny, gx, gy) && c_min < best_goal_cost_) {
      best_goal_node_idx_ = new_idx;
      best_goal_cost_     = c_min;
      RCLCPP_INFO(node->get_logger(),
        "RRTPlanner*: goal reached/improved (iter=%d cost=%.3f nodes=%zu) "
        "– rewiring %d more iter",
        i+1, best_goal_cost_, tree_.size(), post_goal_iterations_);
    }

    // Animacja
    if (viz_publish_every_n_ > 0 && nodes_since_pub >= viz_publish_every_n_) {
      publishTree(node->now());
      nodes_since_pub = 0;
      if (viz_step_delay_ms_ > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(viz_step_delay_ms_));
      }
    }
  }

  publishTree(node->now());

  if (best_goal_node_idx_ == -1) {
    RCLCPP_WARN(node->get_logger(),
      "RRTPlanner*: goal not reached (%d iter, %zu nodes – will continue next call).",
      max_iterations_, tree_.size());
    return path;
  }

  RCLCPP_INFO(node->get_logger(),
    "RRTPlanner*: path cost=%.3f  nodes=%zu  frozen=%s",
    best_goal_cost_, tree_.size(), planning_done_ ? "YES" : "NO");

  return tracePath(best_goal_node_idx_, goal, global_frame_, node->now());
}

// ──────────────────────────────────────────────────────────────────────────────
// nearRadius / near / edgeCost
// ──────────────────────────────────────────────────────────────────────────────

double RRTPlanner::nearRadius() const
{
  // POPRAWKA: nie przycinamy do step_size_ – to blokowało rewiring!
  // Wzór z Karaman & Frazzoli 2011 (d=2):
  //   r = min( gamma * sqrt(log(n)/n),  max_near_radius_ )
  // step_size_ NIE jest górnym ograniczeniem promienia – to jest długość kroku,
  // a nie zasięg sąsiedztwa. Oddzielamy te dwa pojęcia.
  double n = static_cast<double>(tree_.size());
  if (n < 2.0) { return max_near_radius_; }
  double r = gamma_rrt_star_ * std::sqrt(std::log(n) / n);
  return std::min(r, max_near_radius_);
}

std::vector<int> RRTPlanner::near(double x, double y, double radius) const
{
  std::vector<int> res; double r2 = radius * radius;
  for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
    double dx = tree_[i].x-x, dy = tree_[i].y-y;
    if (dx*dx + dy*dy <= r2) { res.push_back(i); }
  }
  return res;
}

double RRTPlanner::edgeCost(const RRTNode & a, const RRTNode & b)
{ return std::hypot(b.x-a.x, b.y-a.y); }

// ──────────────────────────────────────────────────────────────────────────────
// tracePath
// ──────────────────────────────────────────────────────────────────────────────

nav_msgs::msg::Path RRTPlanner::tracePath(
  int goal_node_idx,
  const geometry_msgs::msg::PoseStamped & goal_ps,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id; path.header.stamp = stamp;

  std::vector<int> idx;
  for (int i = goal_node_idx; i != -1; i = tree_[i].parent_idx) { idx.push_back(i); }
  std::reverse(idx.begin(), idx.end());

  path.poses.reserve(idx.size() + 1);
  for (size_t i = 0; i < idx.size(); ++i) {
    const RRTNode & cur = tree_[idx[i]];
    double nx = (i+1 < idx.size()) ? tree_[idx[i+1]].x : goal_ps.pose.position.x;
    double ny = (i+1 < idx.size()) ? tree_[idx[i+1]].y : goal_ps.pose.position.y;
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = cur.x; ps.pose.position.y = cur.y; ps.pose.position.z = 0.0;
    ps.pose.orientation = yawToQuat(std::atan2(ny-cur.y, nx-cur.x));
    path.poses.push_back(ps);
  }
  geometry_msgs::msg::PoseStamped g = goal_ps; g.header = path.header;
  path.poses.push_back(g);
  return path;
}

// ──────────────────────────────────────────────────────────────────────────────
// publishTree / clearTreeMarkers
// ──────────────────────────────────────────────────────────────────────────────

void RRTPlanner::publishTree(const rclcpp::Time & stamp) const
{
  if (!tree_pub_ || !tree_pub_->is_activated() || tree_.empty()) { return; }

  visualization_msgs::msg::MarkerArray ma;

  // ── Marker 0: węzły (SPHERE_LIST) – cyan ─────────────────────────────────
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_; m.header.stamp = stamp;
    m.ns = "rrt_nodes"; m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = node_scale_;
    m.color.r = node_r_; m.color.g = node_g_; m.color.b = node_b_; m.color.a = node_a_;
    m.pose.orientation.w = 1.0;
    for (const auto & n : tree_) {
      geometry_msgs::msg::Point p; p.x=n.x; p.y=n.y; p.z=0.02; m.points.push_back(p);
    }
    ma.markers.push_back(m);
  }

  // ── Marker 1: krawędzie drzewa (LINE_LIST) – niebieski ───────────────────
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_; m.header.stamp = stamp;
    m.ns = "rrt_edges"; m.id = 1;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = edge_scale_;
    m.color.r = edge_r_; m.color.g = edge_g_; m.color.b = edge_b_; m.color.a = edge_a_;
    m.pose.orientation.w = 1.0;
    for (int i = 1; i < static_cast<int>(tree_.size()); ++i) {
      const RRTNode & c = tree_[i], & p = tree_[c.parent_idx];
      geometry_msgs::msg::Point pc, pp;
      pp.x=p.x; pp.y=p.y; pp.z=0.02; pc.x=c.x; pc.y=c.y; pc.z=0.02;
      m.points.push_back(pp); m.points.push_back(pc);
    }
    ma.markers.push_back(m);
  }

  // ── Marker 2: aktualna najlepsza ścieżka (LINE_STRIP) – CZERWONY ─────────
  // Rysowany tylko gdy cel został już znaleziony.
  // Przy każdym rewiringu który poprawia koszt, ten marker automatycznie
  // pokazuje nową (lepszą) ścieżkę, bo śledzi best_goal_node_idx_.
  if (best_goal_node_idx_ != -1) {
    // Zbierz węzły ścieżki: od korzenia do węzła celu
    std::vector<int> path_idx;
    for (int i = best_goal_node_idx_; i != -1; i = tree_[i].parent_idx) {
      path_idx.push_back(i);
    }
    std::reverse(path_idx.begin(), path_idx.end());

    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_; m.header.stamp = stamp;
    m.ns     = "rrt_best_path"; m.id = 2;
    m.type   = visualization_msgs::msg::Marker::LINE_STRIP;  // ciągła linia
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = edge_scale_ * 3.0;   // grubsza niż krawędzie drzewa
    m.color.r = 1.0f;                 // czerwony
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;                 // pełna nieprzezroczystość
    m.pose.orientation.w = 1.0;

    m.points.reserve(path_idx.size());
    for (int idx : path_idx) {
      geometry_msgs::msg::Point p;
      p.x = tree_[idx].x; p.y = tree_[idx].y; p.z = 0.03; // minimalnie wyżej niż drzewo
      m.points.push_back(p);
    }
    ma.markers.push_back(m);
  } else {
    // Cel jeszcze nie znaleziony – wyczyść czerwony marker jeśli był
    visualization_msgs::msg::Marker del;
    del.header.frame_id = global_frame_; del.header.stamp = stamp;
    del.ns = "rrt_best_path"; del.id = 2;
    del.action = visualization_msgs::msg::Marker::DELETE;
    ma.markers.push_back(del);
  }

  tree_pub_->publish(ma);
}


// ──────────────────────────────────────────────────────────────────────────────
// propagateCosts  –  aktualizuje koszty wszystkich potomków przepiętego węzła
// ──────────────────────────────────────────────────────────────────────────────
// POPRAWKA: bez tego po rewiringu dzieci mają nieaktualne koszty, co powoduje
// że warunek c_through_new < x_near.cost nigdy nie zachodzi dla węzłów głębiej
// w drzewie (w tym węzła celu), więc czerwona ścieżka nigdy się nie przepina.
void RRTPlanner::propagateCosts(int node_idx)
{
  // Iteracyjna wersja BFS żeby uniknąć rekurencji na głębokich drzewach
  std::vector<int> queue = {node_idx};
  while (!queue.empty()) {
    int parent = queue.back();
    queue.pop_back();
    // Znajdź wszystkich bezpośrednich potomków tego węzła
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
      if (tree_[i].parent_idx == parent) {
        // Zaktualizuj koszt dziecka na podstawie aktualnego kosztu rodzica
        double new_cost = tree_[parent].cost +
                          std::hypot(tree_[i].x - tree_[parent].x,
                                     tree_[i].y - tree_[parent].y);
        if (new_cost < tree_[i].cost) {
          tree_[i].cost = new_cost;
          // Jeśli to węzeł celu – zaktualizuj best_goal_cost_
          if (i == best_goal_node_idx_) {
            best_goal_cost_ = new_cost;
          }
          queue.push_back(i);   // propaguj dalej w dół
        }
      }
    }
  }
}

void RRTPlanner::clearTreeMarkers(const rclcpp::Time & stamp) const
{
  if (!tree_pub_ || !tree_pub_->is_activated()) { return; }
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker del;
  del.header.frame_id = global_frame_; del.header.stamp = stamp;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  ma.markers.push_back(del); tree_pub_->publish(ma);
}

// ──────────────────────────────────────────────────────────────────────────────
// Pozostałe funkcje
// ──────────────────────────────────────────────────────────────────────────────

std::pair<double,double> RRTPlanner::sampleFree(double gx, double gy)
{
  if (uniform01_(rng_) < goal_bias_) { return {gx, gy}; }
  const double ox=costmap_->getOriginX(), oy=costmap_->getOriginY();
  std::uniform_real_distribution<double> dx(ox, ox+costmap_->getSizeInMetersX());
  std::uniform_real_distribution<double> dy(oy, oy+costmap_->getSizeInMetersY());
  for (int a = 0; a < 100; ++a) {
    double x=dx(rng_), y=dy(rng_); unsigned int mx,my;
    if (!worldToMap(x,y,mx,my)) { continue; }
    if (costmap_->getCost(mx,my) < nav2_costmap_2d::LETHAL_OBSTACLE) { return {x,y}; }
  }
  return {gx, gy};
}

int RRTPlanner::nearest(double x, double y) const
{
  int b=0; double bd=std::numeric_limits<double>::max();
  for (int i=0; i<static_cast<int>(tree_.size()); ++i) {
    double d=(tree_[i].x-x)*(tree_[i].x-x)+(tree_[i].y-y)*(tree_[i].y-y);
    if (d<bd) { bd=d; b=i; }
  }
  return b;
}

std::pair<double,double> RRTPlanner::steer(double fx,double fy,double tx,double ty) const
{
  double dx=tx-fx, dy=ty-fy, dist=std::hypot(dx,dy);
  if (dist<=step_size_) { return {tx,ty}; }
  double s=step_size_/dist; return {fx+dx*s, fy+dy*s};
}

bool RRTPlanner::obstacleFree(double x0,double y0,double x1,double y1) const
{
  double dx=x1-x0, dy=y1-y0, dist=std::hypot(dx,dy);
  int steps=std::max(1,static_cast<int>(std::ceil(dist/collision_check_step_)));
  for (int s=0; s<=steps; ++s) {
    double t=static_cast<double>(s)/steps; unsigned int mx,my;
    if (!worldToMap(x0+t*dx, y0+t*dy, mx, my)) { return false; }
    if (costmap_->getCost(mx,my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) { return false; }
  }
  return true;
}

bool RRTPlanner::goalReached(double x,double y,double gx,double gy) const
{ return std::hypot(x-gx,y-gy)<=goal_tolerance_; }

bool RRTPlanner::worldToMap(double wx,double wy,unsigned int &mx,unsigned int &my) const
{ return costmap_->worldToMap(wx,wy,mx,my); }

bool RRTPlanner::goalChanged(double gx,double gy) const
{
  if (std::isnan(last_goal_x_)||std::isnan(last_goal_y_)) { return true; }
  return std::hypot(gx-last_goal_x_,gy-last_goal_y_)>goal_tolerance_;
}

}  // namespace rrt_planner