#include "hw_omni_controller/planner.hpp"
#include "hw_omni_controller/ros_samples.hpp"
#include <nav2_core/planner_exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>

namespace hw_omni {
void Planner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map){
  node_=parent.lock();if(!node_)throw std::runtime_error("Planner lifecycle node expired");
  tf_=std::move(tf);map_=std::move(map);
  if(map_->getGlobalFrameID()!="map")throw std::invalid_argument("HW planner requires map frame");
  auto parameter=[&](const std::string& key,double value){auto full=name+"."+key;if(!node_->has_parameter(full))node_->declare_parameter(full,value);return node_->get_parameter(full).as_double();};
  double speed=parameter("max_speed_mps",.5),acceleration=parameter("acceleration_mps2",.3);
  radius_=parameter("collision_radius_m",.317658);
  if(!std::isfinite(speed)||speed<=0||speed>.5||!std::isfinite(acceleration)||acceleration<=0||!std::isfinite(radius_)||radius_<=0)throw std::invalid_argument("Invalid HW planner limits");
  // Curvature describes the direction of translation, never radar/chassis yaw.
  nav_executor::CapabilityProfile profile{{{0,speed*.9},{-20,20}},{acceleration,50,acceleration}};
  config_=planner_defaults(profile,{profile,profile,profile});
  config_.endpoint_goal_reached_distance=0;config_.endpoint_nudge_max_distance=0;
  config_.occupied_cost_threshold=253;
  config_.kino_a_star.dynamics.velocity_max=speed;
  config_.kino_a_star.dynamics.tangential_acceleration_max=acceleration;
  config_.kino_a_star.dynamics.lateral_acceleration_max=acceleration;
  config_.kino_a_star.start_yaw_relaxation={16,0,0,speed};
  config_.speed_profile.stationary_velocity_threshold=.03;
  mask_=std::make_shared<nav_executor::RouteTerrainMask>(nav_executor::RouteTerrainMaskParams{.6,.6,1.});
  planner_=std::make_unique<nav_executor::PathPlanner>(config_,mask_,node_->get_logger());
  publisher_=node_->create_publisher<TrajectoryMessage>("/hw/planned_trajectory",rclcpp::QoS(8).transient_local());
  motion_=MotionGate(parameter("lio_timeout_s",.15),parameter("lio_velocity_std_limit_mps",.5));
  odom_sub_=node_->create_subscription<nav_msgs::msg::Odometry>("/navcar/odom",10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg){std::lock_guard<std::mutex> lock(motion_mutex_);motion_.update(motion_sample(*msg),node_->now().seconds());});
}
void Planner::cleanup(){planner_.reset();mask_.reset();publisher_.reset();odom_sub_.reset();}
void Planner::activate(){publisher_->on_activate();}
void Planner::deactivate(){publisher_->on_deactivate();}
nav_msgs::msg::Path Planner::createPlan(const geometry_msgs::msg::PoseStamped& start,const geometry_msgs::msg::PoseStamped& goal,std::function<bool()> canceled){
  auto check=[&]{if(canceled&&canceled())throw nav2_core::PlannerCancelled("HW planning canceled");};check();
  if(start.header.frame_id!="map"||goal.header.frame_id!="map")throw nav2_core::PlannerException("HW planning expects map endpoints");
  U a(start.pose.position.x,start.pose.position.y),b(goal.pose.position.x,goal.pose.position.y);
  if(!a.allFinite()||!b.allFinite())throw nav2_core::PlannerException("Nonfinite planning endpoint");
  Grid grid;
  {auto* map=map_->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    grid.width=map->getSizeInCellsX();grid.height=map->getSizeInCellsY();grid.origin=U(map->getOriginX(),map->getOriginY());grid.resolution=map->getResolution();
    if(grid.width<=0||grid.height<=0||size_t(grid.width)*grid.height>4000000)throw nav2_core::PlannerException("Invalid global grid dimensions");
    grid.cells.assign(map->getCharMap(),map->getCharMap()+size_t(grid.width)*grid.height);}
  // Nav2 inflates obstacles, but not unknown cells. Buffer unknown space and map
  // boundaries for the whole circular vehicle before any search/optimization.
  cv::Mat known(grid.height,grid.width,CV_8UC1);
  for(int y=0;y<grid.height;++y)for(int x=0;x<grid.width;++x)known.at<uint8_t>(y,x)=grid.cells[size_t(y)*grid.width+x]==255?0:255;
  cv::Mat distance;cv::distanceTransform(known,distance,cv::DIST_L2,cv::DIST_MASK_PRECISE);
  auto cells=grid.cells;double guard=radius_+grid.resolution;
  for(int y=0;y<grid.height;++y)for(int x=0;x<grid.width;++x){
    if(distance.at<float>(y,x)*grid.resolution<=guard||std::min({x+.5,y+.5,grid.width-x-.5,grid.height-y-.5})*grid.resolution<=guard)cells[size_t(y)*grid.width+x]=255;
  }
  auto geometry=nav_executor::GridGeometry(grid.width,grid.height,grid.resolution,grid.origin);
  auto cost=std::make_shared<nav_executor::CostMap>(geometry,std::move(cells));
  auto direction=std::make_shared<nav_executor::DirectionMap>(geometry,std::vector<U>(grid.cells.size(),U::Zero()),std::vector<uint8_t>(grid.cells.size(),0));
  if((a-b).norm()<.03){
    if(grid.at(a)>=253||grid.at(b)>=253||!grid.body_known(a,guard)||!grid.body_known(b,guard))throw nav2_core::NoValidPathCouldBeFound("Near goal is occupied/unknown");
    nav_msgs::msg::Path path;path.header.frame_id="map";path.header.stamp=node_->now();
    for(U point:{a,b}){geometry_msgs::msg::PoseStamped p;p.header=path.header;p.pose.position.x=point.x();p.pose.position.y=point.y();p.pose.orientation.w=1;path.poses.push_back(p);}return path;
  }
  mask_->initialize(*cost,direction);check();
  nav_executor::PlanRequest request;request.goal={++generation_,b,false};request.plan_generation=generation_;request.current_pos_map=a;
  U velocity=U::Zero();
  {std::lock_guard<std::mutex> lock(motion_mutex_);if(!motion_.ready(node_->now().seconds()))throw nav2_core::PlannerException("FAST-LIO velocity unavailable or uncertain");
    try{auto tf=tf_->lookupTransform("map",map_->getBaseFrameID(),tf2::TimePointZero);const auto& p=tf.transform.translation;const auto& q=tf.transform.rotation;
      velocity=project_motion(*motion_.sample(),Eigen::Vector3d(p.x,p.y,p.z),Eigen::Quaterniond(q.w,q.x,q.y,q.z),seconds(tf.header.stamp),node_->now().seconds()).velocity;
    }catch(const std::exception&){throw nav2_core::PlannerException("Cannot align FAST-LIO velocity to map pose");}
  }
  U heading=velocity.norm()>.03?velocity:(b-a);
  request.current_yaw=heading.norm()>1e-9?std::atan2(heading.y(),heading.x()):0;
  request.current_velocity=velocity.norm();request.obstacles.global_static=cost;request.obstacles.hard_cost=cost;request.direction_map=direction;
  planner_->set_cancel_checker(canceled);
  nav_executor::PlanResult result;
  try{result=planner_->plan_snapshot(request);}catch(const std::exception&){check();throw;}
  check();
  if(result.kind!=nav_executor::PlanResult::Kind::PATH||!result.path)throw nav2_core::NoValidPathCouldBeFound(result.failure_reason.empty()?"HW planner produced no trajectory":result.failure_reason);
  nav_msgs::msg::Path path;path.header.frame_id="map";path.header.stamp=node_->now();
  double length=result.path->trajectory.total_arc_length();int count=std::max(1,int(std::ceil(length/std::min(.05,grid.resolution*.5))));
  if(count>99999)throw nav2_core::NoValidPathCouldBeFound("HW path too long");
  for(int i=0;i<=count;++i){check();auto sample=result.path->trajectory.eval_arc_length(length*i/count);
    if(grid.at(sample.p)>=253||!grid.body_known(sample.p,guard))throw nav2_core::NoValidPathCouldBeFound("HW final body sweep intersects occupied/unknown space");
    geometry_msgs::msg::PoseStamped p;p.header=path.header;p.pose.position.x=sample.p.x();p.pose.position.y=sample.p.y();p.pose.orientation.w=1;path.poses.push_back(p);}
  publisher_->publish(encode_trajectory(*result.path,path));return path;
}
}
PLUGINLIB_EXPORT_CLASS(hw_omni::Planner,nav2_core::GlobalPlanner)
