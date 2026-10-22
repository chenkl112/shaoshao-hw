#pragma once
#include "tasks.hpp"
#include "ros.hpp"
#include "http.hpp"
#include <hw_omni_controller/feedback.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <shaoshao_hw_interfaces/msg/chassis_feedback.hpp>
#include <shaoshao_hw_interfaces/msg/navigation_session.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace hw_runtime {
// One executor thread owns all ROS callbacks. Tasks' mutex also protects data
// accessed by the local RPC and controller polling threads.
class Navigation {
  using Navigate=nav2_msgs::action::NavigateToPose;
  using Plan=nav2_msgs::action::ComputePathToPose;
  using GoalHandle=rclcpp_action::ClientGoalHandle<Navigate>;
public:
  Navigation(rclcpp::Node::SharedPtr node,Tasks& tasks,fs::path root)
      :node_(std::move(node)),tasks_(tasks),root_(std::move(root)),
       config_(root_/"shaoshao/web_panel/robot.yaml"),gate_(config_.feedback_timeout),
       buffer_(node_->get_clock()),listener_(buffer_){
    navigator_=rclcpp_action::create_client<Navigate>(node_,"/navigate_to_pose");
    planner_=rclcpp_action::create_client<Plan>(node_,"/compute_path_to_pose");
    lifecycle_=node_->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/navigation_lifecycle/manage_nodes");
    odom_pub_=node_->create_publisher<nav_msgs::msg::Odometry>("/navcar/odom",10);
    session_pub_=node_->create_publisher<shaoshao_hw_interfaces::msg::NavigationSession>("/hw/navigation_session",1);
    speed_pub_=node_->create_publisher<nav2_msgs::msg::SpeedLimit>("/navcar/speed_limit",10);
    tasks_.hooks.plan=[this](uint64_t id,const Json::Value& goal){plan(id,goal);};
    tasks_.hooks.goal=[this](uint64_t id,const Json::Value& goal){navigate(id,goal);};
    tasks_.hooks.cancel=[this]{cancel();};
    localization_=node_->create_subscription<std_msgs::msg::String>("/hw/localization/status",10,[this](std_msgs::msg::String::SharedPtr msg){
      try{
        auto value=json_parse(msg->data);double age=(node_->now().nanoseconds()-value["stamp_ns"].asInt64())*1e-9;
        auto state=value["state"].asString();double limit=finite_number(value,"speed_limit_mps");
        if(age<-.05||age>.5||!value["ready"].isBool()||limit<0||limit>.5||
           (state!="acquiring"&&state!="tracking"&&state!="degraded"&&state!="lost"))return;
        std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);
        if(tasks_.mode!="navigation"||tasks_.switching)return;
        quality_at_=steady();tasks_.localization_state(value);
      }catch(const std::exception&){}
    });
    feedback_=node_->create_subscription<shaoshao_hw_interfaces::msg::ChassisFeedback>("/hw/chassis_feedback",10,[this](shaoshao_hw_interfaces::msg::ChassisFeedback::SharedPtr msg){
      hw_omni::ChassisSample sample;sample.stamp=seconds(msg->header.stamp);sample.sequence=msg->seq;sample.authority_epoch=msg->authority_epoch;
      sample.control_allowed=msg->control_allowed;sample.vx=msg->vx_mps;sample.vy=msg->vy_mps;sample.omega=msg->chassis_omega_radps;sample.gimbal_yaw=msg->gimbal_relative_yaw_rad;sample.frame=msg->header.frame_id;
      std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);gate_.update(sample,node_->now().seconds());
      tasks_.hardware(gate_.ready(node_->now().seconds()),gate_.sample()?gate_.sample()->authority_epoch:0);
    });
    odometry_=node_->create_subscription<nav_msgs::msg::Odometry>("/navcar/state_estimation",10,[this](nav_msgs::msg::Odometry::SharedPtr msg){odom(*msg,true);});
    mapping_odom_=node_->create_subscription<nav_msgs::msg::Odometry>("/state_estimation",10,[this](nav_msgs::msg::Odometry::SharedPtr msg){odom(*msg,false);});
    cloud_=node_->create_subscription<std_msgs::msg::String>("/hw/web/cloud_status",10,[this](std_msgs::msg::String::SharedPtr msg){
      try{auto value=json_parse(msg->data);double age=(node_->now().nanoseconds()-value["stamp_ns"].asInt64())*1e-9;
        std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);
        if(age>=-.05&&age<=.5&&value["source"].asString()=="mid360_registered"&&value["count"].asUInt()>0)tasks_.cloud_at=steady()-std::max(0.,age);
      }catch(const std::exception&){}
    });
    map_=node_->create_subscription<nav_msgs::msg::OccupancyGrid>("/projected_map",rclcpp::QoS(1).transient_local(),[this](nav_msgs::msg::OccupancyGrid::SharedPtr msg){
      std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);
      if(tasks_.mode!="mapping"||!fresh(node_->now().seconds(),msg->header.stamp,2)||msg->header.frame_id!="map")return;
      try{auto origin=ros_pose(msg->info.origin);save_grid(tasks_.maps,msg->info.width,msg->info.height,msg->info.resolution,
            Point(origin.translation().x(),origin.translation().y(),std::atan2(origin.linear()(1,0),origin.linear()(0,0))),msg->data);}
      catch(const std::exception& e){RCLCPP_WARN(node_->get_logger(),"Grid save: %s",e.what());}
    });
    motion_=node_->create_subscription<std_msgs::msg::String>("/hw/command/status",1,[this](std_msgs::msg::String::SharedPtr msg){
      try{auto v=json_parse(msg->data);double age=(node_->now().nanoseconds()-v["stamp_ns"].asInt64())*1e-9;
        if(age<-.05||age>.5)return;std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);tasks_.motion=v;
      }catch(const std::exception&){}
    });
    costmap_=node_->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap",rclcpp::QoS(1).transient_local(),[this](nav_msgs::msg::OccupancyGrid::SharedPtr msg){
      if(msg->header.frame_id!="map"||uint64_t(msg->info.width)*msg->info.height>16000000||msg->data.size()!=size_t(msg->info.width)*msg->info.height)return;
      try{auto origin=ros_pose(msg->info.origin);Json::Value v;v["available"]=true;v["frame"]="map";v["resolution"]=msg->info.resolution;v["width"]=msg->info.width;v["height"]=msg->info.height;
        v["origin"]=point_json(Point(origin.translation().x(),origin.translation().y(),std::atan2(origin.linear()(1,0),origin.linear()(0,0))));v["blocked"]=Json::Value(Json::arrayValue);v["lethal"]=Json::Value(Json::arrayValue);
        for(size_t i=0;i<msg->data.size();++i){if(msg->data[i]==99)v["blocked"].append(Json::UInt(i));if(msg->data[i]==100)v["lethal"].append(Json::UInt(i));}
        std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);tasks_.costmap=std::move(v);
      }catch(const std::exception&){}
    });
    path_=node_->create_subscription<nav_msgs::msg::Path>("/plan",10,[this](nav_msgs::msg::Path::SharedPtr msg){
      std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);
      if(!tasks_.active()||msg->header.frame_id!="map"||seconds(msg->header.stamp)<goal_stamp_||msg->poses.empty())return;
      const auto& last=msg->poses.back().pose.position;
      if(std::hypot(last.x-tasks_.goal["x"].asDouble(),last.y-tasks_.goal["y"].asDouble())>.3)return;
      Json::Value path(Json::arrayValue);size_t step=std::max(size_t(1),(msg->poses.size()+1999)/2000);
      for(size_t i=0;i<msg->poses.size();i+=step){const auto& p=msg->poses[i].pose.position;if(!std::isfinite(p.x)||!std::isfinite(p.y))return;Json::Value point(Json::arrayValue);point.append(p.x);point.append(p.y);path.append(point);}tasks_.path=path;
    });
    timer_=node_->create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
  }
  ~Navigation(){release();}
  // Called with Tasks locked, after a mode transition completed.
  void reset(){++mode_epoch_;startup_=false;quality_at_=0;tasks_.costmap["available"]=false;tasks_.motion=Json::Value();release();}
private:
  rclcpp::Node::SharedPtr node_;Tasks& tasks_;fs::path root_;Config config_;hw_omni::FeedbackGate gate_;
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  rclcpp_action::Client<Navigate>::SharedPtr navigator_;rclcpp_action::Client<Plan>::SharedPtr planner_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr lifecycle_;
  GoalHandle::SharedPtr goal_handle_;std::string lease_;std::optional<uint64_t> manual_epoch_;
  double quality_at_=0,goal_stamp_=0;bool startup_=false;uint64_t mode_epoch_=0;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<shaoshao_hw_interfaces::msg::NavigationSession>::SharedPtr session_pub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_,cloud_,motion_;
  rclcpp::Subscription<shaoshao_hw_interfaces::msg::ChassisFeedback>::SharedPtr feedback_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_,mapping_odom_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_,costmap_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_;rclcpp::TimerBase::SharedPtr timer_;
public:
  // HTTP polling runs outside the ROS executor so a slow UART service cannot
  // delay feedback or task heartbeats.
  void poll_serial(){
    try{
      auto v=request("/api/state");std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);
      tasks_.serial_ready=v["connected"].asBool()&&v["error"].asString().empty();tasks_.serial_at=steady();
      uint64_t epoch=v["manual_epoch"].asUInt64();bool changed=manual_epoch_&&*manual_epoch_!=epoch;manual_epoch_=epoch;
      if(changed||(!lease_.empty()&&v["control_source"].asString()!="navigation")){tasks_.cancel();tasks_.clear_resume();release();}
    }catch(const std::exception&){std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);tasks_.serial_ready=false;if(tasks_.active())tasks_.cancel();}
  }
private:
  Json::Value request(const std::string& path,const std::optional<Json::Value>& body=std::nullopt){
    auto token=read_text(root_/"navcar_move.token",1024);while(!token.empty()&&std::isspace(static_cast<unsigned char>(token.back())))token.pop_back();return local_http(8081,token,path,body);
  }
  void release(){if(lease_.empty())return;Json::Value body;body["lease"]=lease_;lease_.clear();try{request("/api/navigation/release",body);}catch(const std::exception&) {}}
  geometry_msgs::msg::PoseStamped target(const Json::Value& v){geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.header.stamp=node_->now();p.pose=to_ros(pose(v["x"].asDouble(),v["y"].asDouble(),v.get("yaw",0.).asDouble()));return p;}
  void plan(uint64_t id,const Json::Value& goal){
    if(!planner_->action_server_is_ready())throw std::runtime_error("路径规划服务未就绪");
    Plan::Goal g;g.goal=target(goal);g.planner_id="GridBased";g.use_start=false;
    rclcpp_action::Client<Plan>::SendGoalOptions options;
    options.goal_response_callback=[this,id](auto handle){if(!handle)tasks_.plan_result(id,{},"规划目标被拒绝");};
    options.result_callback=[this,id](const auto& result){std::vector<Eigen::Vector2d> points;
      if(result.code==rclcpp_action::ResultCode::SUCCEEDED&&result.result&&result.result->path.header.frame_id=="map")
        for(const auto& p:result.result->path.poses)points.emplace_back(p.pose.position.x,p.pose.position.y);
      tasks_.plan_result(id,points,points.empty()?"路径规划失败":"");};
    planner_->async_send_goal(g,options);
  }
  void navigate(uint64_t id,const Json::Value& goal){
    if(!navigator_->action_server_is_ready())throw std::runtime_error("导航服务未就绪");
    release();lease_=request("/api/navigation/acquire",Json::Value(Json::objectValue))["lease"].asString();
    if(lease_.empty())throw std::runtime_error("控制器未返回导航控制权");
    Navigate::Goal g;g.pose=target(goal);goal_stamp_=seconds(g.pose.header.stamp);
    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    options.goal_response_callback=[this,id](auto handle){
      std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);
      if(id!=tasks_.goal_generation){if(handle)navigator_->async_cancel_goal(handle);return;}
      goal_handle_=handle;tasks_.goal_response(id,bool(handle));if(!handle)release();
    };
    options.feedback_callback=[this,id](auto,const auto feedback){std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);if(id==tasks_.goal_generation&&std::isfinite(feedback->distance_remaining))tasks_.remaining=feedback->distance_remaining;};
    options.result_callback=[this,id](const auto& result){
      std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);if(id!=tasks_.goal_generation)return;
      auto code=result.code;tasks_.goal_result(id,code==rclcpp_action::ResultCode::SUCCEEDED?"succeeded":code==rclcpp_action::ResultCode::CANCELED?"canceled":"failed",code==rclcpp_action::ResultCode::SUCCEEDED?"已到达目标":code==rclcpp_action::ResultCode::CANCELED?"导航已取消":"导航失败");goal_handle_.reset();release();};
    try{navigator_->async_send_goal(g,options);}catch(...){release();throw;}
  }
  void cancel(){if(goal_handle_)navigator_->async_cancel_goal(goal_handle_);}
  void odom(nav_msgs::msg::Odometry msg,bool navigation){
    std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);
    if(tasks_.mode!=(navigation?"navigation":"mapping")||tasks_.switching||!fresh(node_->now().seconds(),msg.header.stamp,.5)||msg.header.frame_id!=(navigation?"navcar_odom":"odom"))return;
    try{
      auto transform=ros_pose(msg.pose.pose);auto offset=config_.center_offset;
      if(navigation){auto frames=YAML::LoadFile((root_/"navigation_runtime/frames.yaml").string());offset=yaml_point(frames["center_offset"]);}
      double yaw=std::atan2(transform.linear()(1,0),transform.linear()(0,0));
      if(navigation){
        transform.translation()+=transform.linear()*offset;msg.pose.pose=to_ros(transform);msg.child_frame_id="navcar_center";
        Point omega(msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z);
        Point velocity(msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z);velocity+=omega.cross(offset);
        if(!omega.allFinite()||!velocity.allFinite())return;
        msg.twist.twist.linear.x=velocity.x();msg.twist.twist.linear.y=velocity.y();msg.twist.twist.linear.z=velocity.z();
        Eigen::Matrix<double,6,6> j=Eigen::Matrix<double,6,6>::Identity();Eigen::Matrix3d skew;skew<<0,-offset.z(),offset.y(),offset.z(),0,-offset.x(),-offset.y(),offset.x(),0;j.block<3,3>(0,3)=-skew;
        Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> twist_cov(msg.twist.covariance.data());twist_cov=(j*twist_cov*j.transpose()).eval();
        j.setIdentity();Point d=transform.linear()*(Point::UnitZ().cross(offset));j.block<3,1>(0,5)=d;
        Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> pose_cov(msg.pose.covariance.data());pose_cov=(j*pose_cov*j.transpose()).eval();
        if(gate_.ready(node_->now().seconds())){msg.twist.twist.linear.x=gate_.sample()->vx;msg.twist.twist.linear.y=gate_.sample()->vy;}
        odom_pub_->publish(msg);
      }
      Json::Value p;p["x"]=transform.translation().x();p["y"]=transform.translation().y();p["z"]=transform.translation().z();p["yaw"]=yaw;p["frame"]=msg.header.frame_id;
      tasks_.local_pose=p;tasks_.odom_at=sample_steady(msg.header.stamp,node_->now().seconds(),steady(),.5);
    }catch(const std::exception&){}
  }
  void tick(){
    std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);double wall=steady(),clock=node_->now().seconds();
    tasks_.hardware(gate_.ready(clock),gate_.sample()?gate_.sample()->authority_epoch:0);
    if(tasks_.mode!="navigation"||tasks_.switching){if(!lease_.empty()){cancel();release();}return;}
    // Cancel and retain the goal before Nav2's controller patience expires.
    // Waiting for the global correction timeout loses the action result first.
    if(wall-quality_at_>1 || !tasks_.odom_at || wall-tasks_.odom_at>.2 ||
       !tasks_.cloud_at || wall-tasks_.cloud_at>.5){
      Json::Value status=tasks_.localization;status["ready"]=false;status["state"]="lost";
      status["message"]="定位或雷达数据更新超时";tasks_.localization_state(status);
    }
    if(tasks_.localized&&!tasks_.stack_ready&&!startup_&&lifecycle_->service_is_ready()){
      startup_=true;auto request=std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();request->command=0;auto epoch=mode_epoch_;
      lifecycle_->async_send_request(request,[this,epoch](rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future){std::lock_guard<std::recursive_mutex> lock(tasks_.mutex);if(epoch!=mode_epoch_||tasks_.mode!="navigation")return;try{tasks_.stack_ready=future.get()->success;}catch(...){tasks_.stack_ready=false;}startup_=false;});
    }
    tasks_.nav_server=navigator_->action_server_is_ready()&&planner_->action_server_is_ready();
    try{auto tf=buffer_.lookupTransform("map","navcar_center",tf2::TimePointZero);double sampled=sample_steady(tf.header.stamp,clock,wall,1.);
      if(sampled){auto p=ros_transform(tf.transform);tasks_.pose["x"]=p.translation().x();tasks_.pose["y"]=p.translation().y();tasks_.pose["yaw"]=std::atan2(p.linear()(1,0),p.linear()(0,0));tasks_.pose["frame"]="map";tasks_.pose_at=sampled;}
    }catch(const std::exception&){}
    nav2_msgs::msg::SpeedLimit limit;limit.header.stamp=node_->now();limit.percentage=false;limit.speed_limit=std::clamp(tasks_.localization.get("speed_limit_mps",.05).asDouble(),.001,config_.speed);speed_pub_->publish(limit);
    if(!lease_.empty()){
      shaoshao_hw_interfaces::msg::NavigationSession session;session.header.stamp=node_->now();session.header.frame_id="navcar_center";session.lease=lease_;session.authority_epoch=tasks_.hardware_epoch;session.enabled=tasks_.enabled(wall);session_pub_->publish(session);
      if(!tasks_.active())release();
    }
    tasks_.tick(wall);
  }
};
}
