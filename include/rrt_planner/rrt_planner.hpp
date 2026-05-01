#ifndef RRT_PLANNER__RRT_PLANNER_HPP_
#define RRT_PLANNER__RRT_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <random>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace rrt_planner
{

struct RRTNode
{
  double x;
  double y;
  int parent_idx;  // -1 for root

  RRTNode(double x_, double y_, int parent = -1)
  : x(x_), y(y_), parent_idx(parent) {}
};

class RRTPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTPlanner() = default;
  ~RRTPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup()    override;
  void activate()   override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // ---------- RRT core ----------
  std::pair<double, double> sampleFree(double goal_x, double goal_y);
  int nearest(double x, double y) const;
  std::pair<double, double> steer(
    double from_x, double from_y, double to_x, double to_y) const;
  bool obstacleFree(double x0, double y0, double x1, double y1) const;
  bool goalReached(double x, double y, double gx, double gy) const;

  /**
   * @brief Trace path from goal node to root.
   *        Each pose orientation is set to face the NEXT waypoint so that
   *        the DWB controller can follow the path correctly.
   */
  nav_msgs::msg::Path tracePath(
    int goal_node_idx,
    double goal_x, double goal_y,
    const geometry_msgs::msg::PoseStamped & goal_pose_stamped,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

  // ---------- visualisation ----------
  void publishTree(const rclcpp::Time & stamp) const;
  void clearTreeMarkers(const rclcpp::Time & stamp) const;

  // ---------- helpers ----------
  bool worldToMap(double wx, double wy,
                  unsigned int & mx, unsigned int & my) const;
  bool goalChanged(double gx, double gy) const;

  /** Convert yaw angle to geometry_msgs::msg::Quaternion */
  static geometry_msgs::msg::Quaternion yawToQuat(double yaw);

  // ---------- members ----------
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  std::string global_frame_;

  // Parameters
  int    max_iterations_{5000};
  double step_size_{0.05};
  double goal_tolerance_{0.15};
  double goal_bias_{0.10};
  double collision_check_step_{0.02};

  // Safety margin from inflation zones [0.0 – 1.0]
  // 0.0 ‒ blokuj tylko strefę INSCRIBED (253) = najmniej konserwatywny
  // 0.5 ‒ blokuj komórki z kosztem > 126 (połowa strefy inflacyjnej)
  // 1.0 ‒ blokuj każdą komórkę z kosztem > 0 (maksymalny dystans od ścian)
  // Formula: threshold = 253 * (1.0 - safety_margin)
  double safety_margin_{0.0};

  // Visualisation parameters
  double node_scale_{0.03};
  double edge_scale_{0.01};
  float  node_r_{0.0f}, node_g_{0.8f}, node_b_{0.8f}, node_a_{0.8f};
  float  edge_r_{0.2f}, edge_g_{0.5f}, edge_b_{1.0f}, edge_a_{0.5f};

  // Last goal – tree reset only when goal changes
  double last_goal_x_{std::numeric_limits<double>::quiet_NaN()};
  double last_goal_y_{std::numeric_limits<double>::quiet_NaN()};

  // RRT tree
  mutable std::vector<RRTNode> tree_;

  // RNG
  mutable std::mt19937 rng_;
  mutable std::uniform_real_distribution<double> uniform01_;

  // Visualisation publisher
  using MarkerArrayPub =
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>;
  std::shared_ptr<MarkerArrayPub> tree_pub_;
};

}  // namespace rrt_planner

#endif  // RRT_PLANNER__RRT_PLANNER_HPP_