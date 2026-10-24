#pragma once
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "planner_defaults.hpp"
#include "trajectory_bridge.hpp"
#include "motion.hpp"
#include <mutex>
namespace hw_omni {
class Planner : public nav2_core::GlobalPlanner {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr&,std::string,std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped&,const geometry_msgs::msg::PoseStamped&,std::function<bool()>) override;
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav_executor::RouteTerrainMask> mask_;
  std::unique_ptr<nav_executor::PathPlanner> planner_;
  nav_executor::PlannerConfig config_{};
  rclcpp_lifecycle::LifecyclePublisher<TrajectoryMessage>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::mutex motion_mutex_;
  MotionGate motion_;
  double radius_=.317658;
  uint64_t generation_=0;
};
}
