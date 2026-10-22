#pragma once
#include <mutex>
#include <optional>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <shaoshao_hw_interfaces/msg/chassis_feedback.hpp>
#include <shaoshao_hw_interfaces/msg/predicted_obstacles.hpp>
#include <std_msgs/msg/string.hpp>
#include "mpc.hpp"
#include "feedback.hpp"

namespace hw_omni {
class Controller : public nav2_core::Controller {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr&,std::string,
      std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  void setPlan(const nav_msgs::msg::Path&) override;
  void setSpeedLimit(const double&,const bool&) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped&,
      const geometry_msgs::msg::Twist&,nav2_core::GoalChecker*) override;
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::string name_,base_,global_;
  Settings settings_;
  FeedbackGate feedback_gate_;
  double speed_limit_=0.5,feedback_timeout_=0.15,prediction_timeout_=0.3,pose_timeout_=0.15;
  std::mutex mutex_;
  nav_msgs::msg::Path plan_;
  shaoshao_hw_interfaces::msg::ChassisFeedback::SharedPtr feedback_;
  shaoshao_hw_interfaces::msg::PredictedObstacles::SharedPtr predictions_;
  rclcpp::Subscription<shaoshao_hw_interfaces::msg::ChassisFeedback>::SharedPtr feedback_sub_;
  rclcpp::Subscription<shaoshao_hw_interfaces::msg::PredictedObstacles>::SharedPtr prediction_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr prediction_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  U previous_command_=U::Zero(),previous_increment_=U::Zero();
  double radar_omega_=0;
  rclcpp::Time last_output_{0,0,RCL_ROS_TIME},last_odom_{0,0,RCL_ROS_TIME};
  uint64_t armed_epoch_=0;
  bool armed_=false;
  void reset_unlocked();
};
}
