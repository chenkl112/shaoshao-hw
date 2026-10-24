#include "hw_omni_controller/controller.hpp"
#include "hw_omni_controller/ros_samples.hpp"
#include <nav2_core/controller_exceptions.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <sstream>
#include <nav2_core/progress_checker.hpp>
#include "hw_omni_controller/navigation_grid.hpp"

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
  authority_timeout_=parameter("authority_timeout_s",0.15);
  motion_=MotionGate(parameter("lio_timeout_s",.15),parameter("lio_velocity_std_limit_mps",.5));
  prediction_timeout_=parameter("prediction_timeout_s",0.3);
  pose_timeout_=parameter("pose_timeout_s",0.15);
  settings_.validate();
  recovery_=std::make_unique<Recovery>(node_->get_logger(),settings_.speed);
  tracker_=std::make_unique<nav_executor::RouteTracker>(nav_executor::RouteTrackerParams{
    .5,2.,.2,.05,6,3.,.2,.3,.15,.8,.3,settings_.speed});
  trajectory_sub_=node_->create_subscription<TrajectoryMessage>("/hw/planned_trajectory",rclcpp::QoS(8).transient_local(),
    [this](TrajectoryMessage::SharedPtr msg){
      std::lock_guard<std::mutex> lock(mutex_);
      try{auto trajectory=decode_trajectory(*msg);
        if(same_plan(msg->path,plan_))trajectory_=trajectory;
        trajectory_cache_.push_back({std::move(msg->path),std::move(trajectory)});
        while(trajectory_cache_.size()>8)trajectory_cache_.pop_front();
      }catch(const std::exception& e){RCLCPP_WARN(node_->get_logger(),"Rejected HW trajectory: %s",e.what());}
    });
  authority_=AuthorityGate(authority_timeout_);
  if(prediction_timeout_<=0||prediction_timeout_>.5||pose_timeout_<=0||pose_timeout_>.15)throw std::invalid_argument("Invalid controller freshness bounds");
  authority_sub_=node_->create_subscription<shaoshao_hw_interfaces::msg::ControlAuthority>("/hw/control_authority",10,
      [this](shaoshao_hw_interfaces::msg::ControlAuthority::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);auto previous=authority_.sample();
        auto update=authority_.update(authority_sample(*msg),node_->now().seconds());
        if(update==SampleUpdate::invalid||(update==SampleUpdate::accepted&&(!msg->control_allowed||(previous&&previous->authority_epoch!=msg->authority_epoch))))reset_unlocked();
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
        auto update=motion_.update(motion_sample(*msg),node_->now().seconds());
        if(update==SampleUpdate::invalid)reset_unlocked();
      });
  prediction_pub_=node_->create_publisher<nav_msgs::msg::Path>("/hw/mpc/prediction",1);
  status_pub_=node_->create_publisher<std_msgs::msg::String>("/hw/mpc/status",1);
}
void Controller::reset_unlocked(){previous_command_.setZero();previous_increment_.setZero();last_output_=rclcpp::Time(0,0,RCL_ROS_TIME);armed_=false;if(recovery_)recovery_->reset();if(tracker_)tracker_->reset();}
void Controller::reset(){std::lock_guard<std::mutex> lock(mutex_);reset_unlocked();}
void Controller::activate(){prediction_pub_->on_activate();status_pub_->on_activate();}
void Controller::deactivate(){std::lock_guard<std::mutex> lock(mutex_);reset_unlocked();prediction_pub_->on_deactivate();status_pub_->on_deactivate();}
void Controller::cleanup(){trajectory_sub_.reset();trajectory_cache_.clear();trajectory_.reset();authority_sub_.reset();prediction_sub_.reset();odom_sub_.reset();prediction_pub_.reset();status_pub_.reset();}
void Controller::setPlan(const nav_msgs::msg::Path& path){
  std::lock_guard<std::mutex> lock(mutex_);
  if(path.header.frame_id!="map"||path.poses.empty())throw nav2_core::InvalidPath("Expected nonempty map path");
  for(const auto& p:path.poses)
    if(!std::isfinite(p.pose.position.x)||!std::isfinite(p.pose.position.y))throw nav2_core::InvalidPath("Nonfinite path");
  if(!same_plan(plan_,path)){
    trajectory_.reset();tracker_->reset();recovery_->reset();plan_at_=node_->now();
    for(auto it=trajectory_cache_.rbegin();it!=trajectory_cache_.rend();++it)if(same_plan(it->path,path)){trajectory_=it->trajectory;break;}
  }
  plan_=path;
  // Nav2 supplies a new plan to re-arm this controller. The web bridge cancels
  // obsolete actions on takeover and independently gates output by its lease.
  if(authority_.ready(node_->now().seconds())){armed_=true;armed_epoch_=authority_.sample()->authority_epoch;}
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
  if(!authority_.ready(now.seconds()))return fail("Hardware control authority unavailable");
  if(!motion_.ready(now.seconds()))return fail("FAST-LIO velocity unavailable or uncertain");
  if(!armed_||armed_epoch_!=authority_.sample()->authority_epoch)return fail("New navigation plan required after authority change");
  if(!predictions_||!predictions_->valid||predictions_->header.frame_id!="map"||!fresh(now,predictions_->header.stamp,prediction_timeout_))return fail("Obstacle prediction unavailable");
  if(!fresh(now,pose.header.stamp,pose_timeout_))return fail("Lidar pose or heading rate stale");
  if(plan_.poses.empty())return fail("No global plan");
  geometry_msgs::msg::PoseStamped mapped=pose;
  try{if(pose.header.frame_id!="map")mapped=tf_->transform(pose,"map",tf2::durationFromSec(0.03));}
  catch(const tf2::TransformException&){return fail("Map transform unavailable");}
  MotionProjection projection;
  try{const auto& p=mapped.pose.position;const auto& q=mapped.pose.orientation;
    projection=project_motion(*motion_.sample(),Eigen::Vector3d(p.x,p.y,p.z),Eigen::Quaterniond(q.w,q.x,q.y,q.z),stamp_seconds(mapped.header.stamp),now.seconds());
  }catch(const std::exception&){return fail("FAST-LIO and map pose projection invalid");}
  double yaw=projection.yaw;U position=projection.position,measured=projection.velocity;radar_omega_=projection.omega;
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
  for(int k=0;k<=N;++k)problem.lidar_yaws[k]=yaw+radar_omega_*k*DT;
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
  auto stopped=[&](const std::string& reason){
    previous_command_.setZero();previous_increment_.setZero();last_output_=now;
    geometry_msgs::msg::TwistStamped out;out.header.frame_id=base_;out.header.stamp=now;
    std_msgs::msg::String status;std::ostringstream text;text<<"{\"ready\":false,\"state\":\""<<motion_name(recovery_->state())<<"\",\"reason\":\""<<reason<<"\"}";status.data=text.str();status_pub_->publish(status);return out;
  };
  const auto& endpoint=plan_.poses.back().pose.position;
  if((position-U(endpoint.x,endpoint.y)).norm()<.03&&!recovery_->active())return stopped("Goal position reached; waiting for measured stop");
  if(!trajectory_){if((now-plan_at_).seconds()>.5)return fail("HW trajectory snapshot unavailable");return stopped("Waiting for matching HW trajectory");}
  double travel_heading=measured.norm()>.01?std::atan2(measured.y(),measured.x()):yaw;
  auto stamp=std::chrono::steady_clock::now();
  auto route=tracker_->update(trajectory_,Eigen::Vector3d(position.x(),position.y(),travel_heading),measured.norm(),stamp);
  nav_executor::GridGeometry geometry(grid.width,grid.height,grid.resolution,grid.origin);
  nav_executor::CostMap recovery_cost(geometry,recovery_cost_cells(grid,settings_.radius));
  nav_executor::DirectionMap flat(geometry,std::vector<U>(grid.cells.size(),U::Zero()),std::vector<uint8_t>(grid.cells.size(),0));
  auto decision=recovery_->update(position,measured,previous,route?&*route:nullptr,recovery_cost,flat,stamp);
  if(decision.replan)return fail("HW recovery completed; replan original goal");
  if(!recovery_->active()){
    if(!route||route->status!=nav_executor::RouteTrackingStatus::TRACKED)return fail("HW route tracking lost; replan original goal");
    if((position-route->reference_position).norm()>.001){
      // HW's projection guard invalidates the route before following a reference
      // across occupied space. Only inspect the segment inside the local map.
      int samples=std::max(20,int(std::ceil((position-route->reference_position).norm()/(grid.resolution*.5))));
      for(int i=0;i<=samples;++i){U p=position+(route->reference_position-position)*double(i)/samples;
        if(grid.at(p)>=250||!grid.body_known(p,settings_.radius))return fail("HW route projection blocked; replan original goal");}
    }
    double progress=route->arc_length,reference_speed=std::min(measured.norm(),speed_limit_);
    for(int k=0;k<=N;++k){
      auto sample=trajectory_->trajectory.eval_arc_length(progress);
      double profile_speed=trajectory_->speed_profile.eval_arc_length(progress).velocity;
      // Start from measured progress speed so a zero initial profile node cannot
      // lock a stationary robot at s=0. Time-law acceleration provides launch.
      double remaining=trajectory_->trajectory.total_arc_length()-progress;
      double target=std::min({speed_limit_*.9,std::sqrt(std::max(0.,2*settings_.acceleration*remaining)),std::max(profile_speed,std::min(.1,settings_.acceleration*(k+1)*DT))});
      reference_speed=std::clamp(target,std::max(0.,reference_speed-settings_.acceleration*DT),reference_speed+settings_.acceleration*DT);
      problem.positions[k]=sample.p;problem.velocities[k]=reference_speed*U(std::cos(sample.theta),std::sin(sample.theta));
      progress=std::min(trajectory_->trajectory.total_arc_length(),progress+reference_speed*DT);
    }
  }else{
    if(!decision.target)return stopped("HW safe-point search has no collision-free candidate");
    U target=*decision.target;
    double recovery_limit=std::min(speed_limit_,recovery_->reverse_speed());
    if(speed_limit_hold(recovery_limit,previous,measured))return stopped("HW recovery waiting for translation to slow");
    problem.settings.speed=recovery_limit;
    if(decision.state==nav_executor::MotionState::STUCK_REVERSE){
      U direction=target-position;double distance=direction.norm();
      for(int k=0;k<=N;++k){double step=std::min(distance,recovery_->reverse_speed()*k*DT);problem.positions[k]=position+(distance>1e-8?U(direction/distance*step):U::Zero());problem.velocities[k]=distance>step&&distance>1e-8?U(direction/distance*recovery_->reverse_speed()):U::Zero();}
    }else{
      for(int k=0;k<=N;++k){problem.positions[k]=target;problem.velocities[k].setZero();}
    }
  }
  X initial=X::Zero();initial.head<2>()=position;initial.segment<2>(2)=measured;
  initial.segment<2>(4)=previous;initial.segment<2>(6)=increment;
  Solution solution;
  try{solution=solve(problem,initial);}catch(const std::exception&){return fail("MPC input or prediction invalid");}
  if(!solution.valid){
    bool collision=solution.reason.find("collision")!=std::string::npos;
    if(collision){
      if(!recovery_->active()&&recovery_->reject_rollout())return fail("HW MPC lethal rollout rejected for 3 cycles; replan original goal");
      return stopped("HW MPC rejected collision rollout; stopping");
    }
    return fail(solution.reason.empty()?"No collision-free feasible MPC rollout":solution.reason);
  }
  recovery_->accept_rollout();
  U command=rotation(-yaw)*solution.command;
  U delta=command-previous_command_;
  previous_command_=command;previous_increment_=delta;last_output_=now;
  geometry_msgs::msg::TwistStamped output;output.header.frame_id=base_;output.header.stamp=now;
  output.twist.linear.x=command.x();output.twist.linear.y=command.y();output.twist.angular.z=0;
  nav_msgs::msg::Path prediction;prediction.header.frame_id="map";prediction.header.stamp=now;
  for(int k=0;k<=N;++k){geometry_msgs::msg::PoseStamped p;p.header=prediction.header;p.header.stamp=now+rclcpp::Duration::from_seconds(k*DT);p.pose.position.x=solution.states[k][0];p.pose.position.y=solution.states[k][1];p.pose.orientation.w=1;prediction.poses.push_back(p);}
  prediction_pub_->publish(prediction);
  std_msgs::msg::String status;std::ostringstream text;text<<"{\"ready\":true,\"velocity_source\":\"fast_lio\",\"state\":\""<<motion_name(recovery_->state())<<"\",\"cost\":"<<solution.cost<<",\"lio_vx\":"<<motion_.sample()->velocity.x()<<",\"lio_vy\":"<<motion_.sample()->velocity.y()<<",\"lidar_omega\":"<<radar_omega_<<"}";status.data=text.str();status_pub_->publish(status);
  return output;
}
}
PLUGINLIB_EXPORT_CLASS(hw_omni::Controller,nav2_core::Controller)

namespace hw_omni {
// Progress is owned by HW's arc-length monitor and recovery state machine.
// Nav2 must not abort a safe-point search because translation is temporarily zero.
class RecoveryProgressChecker : public nav2_core::ProgressChecker {
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr&,const std::string&) override {}
  bool check(geometry_msgs::msg::PoseStamped&) override { return true; }
  void reset() override {}
};
}
PLUGINLIB_EXPORT_CLASS(hw_omni::RecoveryProgressChecker,nav2_core::ProgressChecker)
