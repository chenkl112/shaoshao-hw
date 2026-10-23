#include "hw_omni_controller/controller.hpp"
#include "hw_omni_controller/ros_samples.hpp"
#include <nav2_core/controller_exceptions.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <sstream>

namespace hw_omni {
namespace {
Eigen::Matrix2d rotation(double angle){Eigen::Matrix2d r;double c=std::cos(angle),s=std::sin(angle);r<<c,-s,s,c;return r;}
double stamp_seconds(const builtin_interfaces::msg::Time& t){return t.sec+t.nanosec*1e-9;}
bool fresh(const rclcpp::Time& now,const builtin_interfaces::msg::Time& stamp,double timeout){
  double age=now.seconds()-stamp_seconds(stamp);return age>=-0.05&&age<=timeout;
}
}
void Controller::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map){
  node_=parent.lock();if(!node_)throw std::runtime_error("Controller lifecycle node expired");
  name_=std::move(name);tf_=std::move(tf);costmap_=std::move(map);
  base_=costmap_->getBaseFrameID();global_=costmap_->getGlobalFrameID();
  auto parameter=[&](const std::string& key,double value){
    std::string full=name_+"."+key;
    if(!node_->has_parameter(full))node_->declare_parameter(full,value);
    return node_->get_parameter(full).as_double();
  };
  settings_.tau_x=parameter("tau_x_s",0.3);settings_.tau_y=parameter("tau_y_s",0.3);
  settings_.speed=parameter("max_speed_mps",0.5);speed_limit_=settings_.speed;
  settings_.acceleration=parameter("acceleration_mps2",0.3);settings_.jerk=parameter("jerk_mps3",1.0);
  settings_.radius=parameter("collision_radius_m",0.317658);
  settings_.obstacle_margin=parameter("obstacle_margin_m",0.15);
  feedback_timeout_=parameter("feedback_timeout_s",0.15);
  prediction_timeout_=parameter("prediction_timeout_s",0.3);
  pose_timeout_=parameter("pose_timeout_s",0.15);
  settings_.validate();
  feedback_gate_=FeedbackGate(feedback_timeout_);
  if(feedback_timeout_<=0||feedback_timeout_>0.2||prediction_timeout_<=0||prediction_timeout_>0.5||pose_timeout_<=0||pose_timeout_>0.3)
    throw std::invalid_argument("Invalid controller freshness bounds");
  feedback_sub_=node_->create_subscription<shaoshao_hw_interfaces::msg::ChassisFeedback>("/hw/chassis_feedback",10,
      [this](shaoshao_hw_interfaces::msg::ChassisFeedback::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        auto update=feedback_gate_.update(chassis_sample(*msg),node_->now().seconds());
        if(update==SampleUpdate::invalid){reset_unlocked();feedback_.reset();return;}
        if(update==SampleUpdate::replayed)return;
        if(!msg->control_allowed||(feedback_&&msg->authority_epoch!=feedback_->authority_epoch))reset_unlocked();
        feedback_=std::move(msg);
      });
  prediction_sub_=node_->create_subscription<shaoshao_hw_interfaces::msg::PredictedObstacles>("/hw/predicted_obstacles",1,
      [this](shaoshao_hw_interfaces::msg::PredictedObstacles::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        if(predictions_&&stamp_seconds(msg->header.stamp)<=stamp_seconds(predictions_->header.stamp))return;
        try{read_predictions(*msg,node_->now().seconds());}catch(const std::exception&){predictions_.reset();return;}
        predictions_=std::move(msg);
      });
  odom_sub_=node_->create_subscription<nav_msgs::msg::Odometry>("/navcar/odom",10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        if(msg->child_frame_id!=base_||!fresh(node_->now(),msg->header.stamp,pose_timeout_)||!std::isfinite(msg->twist.twist.angular.z))return;
        last_odom_=rclcpp::Time(msg->header.stamp);radar_omega_=msg->twist.twist.angular.z;
      });
  prediction_pub_=node_->create_publisher<nav_msgs::msg::Path>("/hw/mpc/prediction",1);
  status_pub_=node_->create_publisher<std_msgs::msg::String>("/hw/mpc/status",1);
}
void Controller::reset_unlocked(){previous_command_.setZero();previous_increment_.setZero();last_output_=rclcpp::Time(0,0,RCL_ROS_TIME);armed_=false;}
void Controller::reset(){std::lock_guard<std::mutex> lock(mutex_);reset_unlocked();}
void Controller::activate(){prediction_pub_->on_activate();status_pub_->on_activate();}
void Controller::deactivate(){std::lock_guard<std::mutex> lock(mutex_);reset_unlocked();prediction_pub_->on_deactivate();status_pub_->on_deactivate();}
void Controller::cleanup(){feedback_sub_.reset();prediction_sub_.reset();odom_sub_.reset();prediction_pub_.reset();status_pub_.reset();}
void Controller::setPlan(const nav_msgs::msg::Path& path){
  std::lock_guard<std::mutex> lock(mutex_);
  if(path.header.frame_id!="map"||path.poses.empty())throw nav2_core::InvalidPath("Expected nonempty map path");
  for(const auto& p:path.poses)
    if(!std::isfinite(p.pose.position.x)||!std::isfinite(p.pose.position.y))throw nav2_core::InvalidPath("Nonfinite path");
  plan_=path;
  // Nav2 supplies a new plan to re-arm this controller. The web bridge cancels
  // obsolete actions on takeover and independently gates output by its lease.
  if(feedback_&&feedback_->control_allowed){armed_=true;armed_epoch_=feedback_->authority_epoch;}
}
void Controller::setSpeedLimit(const double& speed,const bool& percentage){
  std::lock_guard<std::mutex> lock(mutex_);
  if(!std::isfinite(speed)||speed<0)throw std::invalid_argument("Invalid speed limit");
  // Nav2 uses zero as NO_SPEED_LIMIT. Localization/output gates independently stop.
  speed_limit_=speed==0?settings_.speed:std::min(settings_.speed,percentage?settings_.speed*speed/100:speed);
}

geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist&,nav2_core::GoalChecker*){
  std::lock_guard<std::mutex> lock(mutex_);
  auto now=node_->now();
  auto fail=[&](const std::string& message)->geometry_msgs::msg::TwistStamped{
    reset_unlocked();std_msgs::msg::String status;status.data="{\"ready\":false,\"reason\":\""+message+"\"}";status_pub_->publish(status);
    throw nav2_core::NoValidControl(message);
  };
  if(!feedback_||!feedback_gate_.ready(now.seconds()))return fail("Actual chassis feedback unavailable");
  if(!armed_||armed_epoch_!=feedback_->authority_epoch)return fail("New navigation plan required after authority change");
  if(!predictions_||!predictions_->valid||predictions_->header.frame_id!="map"||!fresh(now,predictions_->header.stamp,prediction_timeout_))return fail("Obstacle prediction unavailable");
  if(!fresh(now,pose.header.stamp,pose_timeout_)||(now-last_odom_).seconds()>pose_timeout_)return fail("Lidar pose or heading rate stale");
  if(plan_.poses.empty())return fail("No global plan");
  geometry_msgs::msg::PoseStamped mapped=pose;
  try{if(pose.header.frame_id!="map")mapped=tf_->transform(pose,"map",tf2::durationFromSec(0.03));}
  catch(const tf2::TransformException&){return fail("Map transform unavailable");}
  double yaw=tf2::getYaw(mapped.pose.orientation);
  U position(mapped.pose.position.x,mapped.pose.position.y);
  if(!position.allFinite()||!std::isfinite(yaw))return fail("Invalid lidar pose");
  // Feedback can arrive faster than LIO publishes TF. Advance the latest lidar
  // heading with its measured angular rate to the feedback's sample timestamp,
  // rather than blocking for a future TF or substituting chassis spin.
  double pose_stamp=stamp_seconds(mapped.header.stamp),pose_age=now.seconds()-pose_stamp;
  if(pose_age<-.05||pose_age>pose_timeout_)return fail("Map pose stale");
  double feedback_yaw=yaw+radar_omega_*(stamp_seconds(feedback_->header.stamp)-pose_stamp);
  U measured=rotation(feedback_yaw)*U(feedback_->vx_mps,feedback_->vy_mps);
  position+=measured*std::max(0.0,pose_age);yaw+=radar_omega_*std::max(0.0,pose_age);
  double elapsed=last_output_.nanoseconds()==0?DT:(now-last_output_).seconds();
  if(last_output_.nanoseconds()!=0&&(elapsed<0.025||elapsed>0.10))return fail("Control timing discontinuity");
  U previous=rotation(yaw)*previous_command_;
  U increment=rotation(yaw)*previous_increment_;
  if(speed_limit_hold(speed_limit_,previous,measured)){
    previous_command_.setZero();previous_increment_.setZero();last_output_=now;
    geometry_msgs::msg::TwistStamped stopped;stopped.header.frame_id=base_;stopped.header.stamp=now;
    std_msgs::msg::String status;status.data="{\"ready\":false,\"reason\":\"Speed limit braking hold\"}";status_pub_->publish(status);
    return stopped; // Safety stop: retain the plan and authority, not a Nav2 fault.
  }
  Problem problem;problem.settings=settings_;problem.settings.speed=speed_limit_;
  // Construct references from the closest segment, not the closest vertex.
  std::vector<U> points;std::vector<double> cumulative{0};
  for(const auto& p:plan_.poses){U v(p.pose.position.x,p.pose.position.y);if(points.empty()||(v-points.back()).norm()>1e-6){if(!points.empty())cumulative.push_back(cumulative.back()+(v-points.back()).norm());points.push_back(v);}}
  double progress=0,best=std::numeric_limits<double>::infinity();
  for(size_t i=1;i<points.size();++i){U edge=points[i]-points[i-1];double t=std::clamp((position-points[i-1]).dot(edge)/edge.squaredNorm(),0.0,1.0);double d=(position-points[i-1]-t*edge).squaredNorm();if(d<best){best=d;progress=cumulative[i-1]+t*edge.norm();}}
  auto reference=[&](double distance){
    if(points.size()==1||distance>=cumulative.back())return std::pair<U,U>{points.back(),U::Zero()};
    auto i=static_cast<size_t>(std::upper_bound(cumulative.begin(),cumulative.end(),distance)-cumulative.begin());
    i=std::clamp(i,size_t(1),points.size()-1);U direction=(points[i]-points[i-1]).normalized();
    return std::pair<U,U>{points[i-1]+direction*(distance-cumulative[i-1]),direction};
  };
  double reference_speed=std::min(measured.norm(),speed_limit_);
  for(int k=0;k<=N;++k){
    double remaining=std::max(0.0,cumulative.back()-progress);
    double target=std::min(speed_limit_*0.90,std::sqrt(2*settings_.acceleration*remaining));
    reference_speed=std::clamp(target,std::max(0.0,reference_speed-settings_.acceleration*DT),reference_speed+settings_.acceleration*DT);
    auto [p,d]=reference(progress);problem.positions[k]=p;problem.velocities[k]=reference_speed*d;
    problem.lidar_yaws[k]=yaw+radar_omega_*k*DT;
    progress=std::min(cumulative.back(),progress+reference_speed*DT);
  }
  double age=now.seconds()-stamp_seconds(predictions_->header.stamp);
  if(predictions_->obstacles.size()>128||!std::isfinite(predictions_->prediction_dt)||predictions_->prediction_dt<0.02||predictions_->prediction_dt>0.2)return fail("Invalid obstacle prediction dimensions");
  for(const auto& item:predictions_->obstacles){
    Obstacle obstacle;obstacle.dt=predictions_->prediction_dt;obstacle.age=std::max(0.0,age);
    if(item.centers.size()<2||item.centers.size()>256||item.footprint.points.size()<3||item.footprint.points.size()>512)return fail("Invalid obstacle footprint or horizon");
    if((item.centers.size()-1)*obstacle.dt<obstacle.age+N*DT)return fail("Obstacle prediction horizon exhausted");
    for(const auto& p:item.centers){if(!std::isfinite(p.x)||!std::isfinite(p.y))return fail("Nonfinite obstacle prediction");obstacle.centers.emplace_back(p.x,p.y);}
    for(const auto& p:item.footprint.points){if(!std::isfinite(p.x)||!std::isfinite(p.y))return fail("Nonfinite obstacle shape");obstacle.footprint.emplace_back(p.x,p.y);}
    problem.obstacles.push_back(std::move(obstacle));
  }
  Grid grid;
  {
    auto* map=costmap_->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
    grid.width=static_cast<int>(map->getSizeInCellsX());grid.height=static_cast<int>(map->getSizeInCellsY());
    grid.resolution=map->getResolution();grid.origin=U(map->getOriginX(),map->getOriginY());
    grid.cells.assign(map->getCharMap(),map->getCharMap()+static_cast<size_t>(grid.width)*grid.height);
  }
  // A map-frame local costmap avoids mixing map predictions and odom cells.
  if(global_!="map")return fail("HW controller requires a map-frame local costmap");
  problem.grid=&grid;
  X initial=X::Zero();initial.head<2>()=position;initial.segment<2>(2)=measured;
  initial.segment<2>(4)=previous;initial.segment<2>(6)=increment;
  Solution solution;
  try{solution=solve(problem,initial);}catch(const std::exception&){return fail("MPC input or prediction invalid");}
  if(!solution.valid)return fail(solution.reason.empty()?"No collision-free feasible MPC rollout":solution.reason);
  U command=rotation(-yaw)*solution.command;
  U delta=command-previous_command_;
  previous_command_=command;previous_increment_=delta;last_output_=now;
  geometry_msgs::msg::TwistStamped output;output.header.frame_id=base_;output.header.stamp=now;
  output.twist.linear.x=command.x();output.twist.linear.y=command.y();output.twist.angular.z=0;
  nav_msgs::msg::Path prediction;prediction.header.frame_id="map";prediction.header.stamp=now;
  for(int k=0;k<=N;++k){geometry_msgs::msg::PoseStamped p;p.header=prediction.header;p.header.stamp=now+rclcpp::Duration::from_seconds(k*DT);p.pose.position.x=solution.states[k][0];p.pose.position.y=solution.states[k][1];p.pose.orientation.w=1;prediction.poses.push_back(p);}
  prediction_pub_->publish(prediction);
  std_msgs::msg::String status;std::ostringstream text;text<<"{\"ready\":true,\"cost\":"<<solution.cost<<",\"measured_vx\":"<<feedback_->vx_mps<<",\"measured_vy\":"<<feedback_->vy_mps<<",\"chassis_omega\":"<<feedback_->chassis_omega_radps<<"}";status.data=text.str();status_pub_->publish(status);
  return output;
}
}
PLUGINLIB_EXPORT_CLASS(hw_omni::Controller,nav2_core::Controller)
