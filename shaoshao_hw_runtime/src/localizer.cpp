#include "shaoshao_runtime/ros.hpp"
#include "shaoshao_runtime/index.hpp"
#include "shaoshao_runtime/policy.hpp"
#include <deque>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <shaoshao_hw_interfaces/srv/register_cloud.hpp>
#include <shaoshao_hw_interfaces/msg/localization_status.hpp>

namespace hw_runtime {
class Localizer : public rclcpp::Node {
  using Registration=shaoshao_hw_interfaces::srv::RegisterCloud;
  struct Frame {double stamp;Cloud points;builtin_interfaces::msg::Time time;};
public:
  Localizer():Node("hw_localizer"),config_(declare_parameter<std::string>("config_path",(runtime_path()/"shaoshao/web_panel/robot.yaml").string())),policy_(config_.registration),maps_(declare_parameter<std::string>("maps_dir","")),index_(maps_/"keyframes",maps_/"site.pcd"){
    auto runtime=fs::path(declare_parameter<std::string>("runtime_root",runtime_path().string()));auto frames=YAML::LoadFile((runtime/"navigation_runtime/frames.yaml").string());
    center_=yaml_point(frames["center_offset"]);lidar_offset_=config_.lidar_offset;
    for(auto t:frames["static_transforms"])if(t["child"].as<std::string>()=="navcar_lidar")lidar_offset_=yaml_point(t["xyz"]);
    lidar_xy_=Eigen::Vector2d(config_.yaml["yaw"]["lidar_offset_xy_m"][0].as<double>(),config_.yaml["yaw"]["lidar_offset_xy_m"][1].as<double>());
    client_=create_client<Registration>("/hw/register");broadcaster_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    status_=create_publisher<std_msgs::msg::String>("/hw/localization/status",10);typed_=create_publisher<shaoshao_hw_interfaces::msg::LocalizationStatus>("/hw/localization/state",10);
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::SharedPtr msg){
      if(msg->header.frame_id!="navcar_odom"||!fresh(now().seconds(),msg->header.stamp,.5))return;double stamp=seconds(msg->header.stamp);if(!frames_.empty()&&stamp<=frames_.back().stamp)return;
      try{frames_.push_back({stamp,decode(*msg),msg->header.stamp});while(frames_.size()>6)frames_.pop_front();}catch(const std::exception&){}
    });
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/navcar/state_estimation",10,[this](nav_msgs::msg::Odometry::SharedPtr msg){
      if(msg->header.frame_id!="navcar_odom"||!fresh(now().seconds(),msg->header.stamp,.5))return;
      try{imu_=ros_pose(msg->pose.pose);auto center=*imu_;center.translation()+=center.linear()*center_;policy_.odometry(center,seconds(msg->header.stamp));}catch(const std::exception&){}
    });
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/navcar/scan",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::SharedPtr msg){
      if(msg->header.frame_id!="navcar_lidar"||!fresh(now().seconds(),msg->header.stamp,config_.registration.scan_timeout))return;std::vector<Eigen::Vector2d> points;
      for(size_t i=0;i<msg->ranges.size();++i){double range=msg->ranges[i];if(std::isfinite(range)&&range>=msg->range_min&&range<msg->range_max){double angle=msg->angle_min+i*msg->angle_increment;points.emplace_back(lidar_xy_+range*Eigen::Vector2d(std::cos(angle),std::sin(angle)));}}
      policy_.scan(seconds(msg->header.stamp),points);
    });
    retry_=create_service<std_srvs::srv::Trigger>("/hw/localization/retry",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response){reset();response->success=true;response->message="自动全局重定位已重新开始";});
    timer_=create_wall_timer(std::chrono::milliseconds(100),std::bind(&Localizer::tick,this));search_started_=steady();
  }
private:
  Config config_;Policy policy_;fs::path maps_;RetrievalIndex index_;Point center_,lidar_offset_;Eigen::Vector2d lidar_xy_;
  std::deque<Frame> frames_;std::optional<Pose> imu_,seed_;std::string mode_;Cloud batch_;Frame latest_;std::deque<Pose> guesses_;std::vector<std::pair<Pose,Metrics>> results_;
  bool pending_=false,batching_=false;uint64_t generation_=0;int64_t request_id_=0;double query_started_=0,last_query_=0,search_started_=0,retry_at_=0;
  rclcpp::Client<Registration>::SharedPtr client_;std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;rclcpp::Publisher<shaoshao_hw_interfaces::msg::LocalizationStatus>::SharedPtr typed_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr retry_;rclcpp::TimerBase::SharedPtr timer_;
  void reset(){policy_.lose("正在重新搜索位置");seed_.reset();retry_at_=0;search_started_=steady();++generation_;if(pending_)client_->remove_pending_request(request_id_);pending_=batching_=false;guesses_.clear();results_.clear();}
  void start_batch(){
    if(frames_.empty()||!imu_||!client_->service_is_ready())return;latest_=frames_.back();if(now().seconds()-latest_.stamp>.5)return;
    Cloud points;for(const auto& frame:frames_)if(latest_.stamp-frame.stamp<=.7)points.insert(points.end(),frame.points.begin(),frame.points.end());if(points.size()<1000)return;
    size_t step=std::max(size_t(1),(points.size()+179999)/180000);batch_.clear();for(size_t i=0;i<points.size();i+=step)batch_.push_back(points[i]);
    mode_=policy_.initialized?"local":seed_?"verify":"global";guesses_.clear();results_.clear();
    if(mode_=="global"){auto sensor=*imu_;sensor.translation()+=sensor.linear()*lidar_offset_;Cloud local;auto inverse=sensor.inverse();for(const auto& p:batch_)local.push_back(inverse*p);for(const auto& guess:index_.candidates(local,sensor))guesses_.push_back(guess);}
    else guesses_.push_back(mode_=="verify"?*seed_:*policy_.correction);batching_=true;++generation_;submit_next();
  }
  void submit_next(){
    if(guesses_.empty()){
      std::optional<std::pair<Pose,Metrics>> best=mode_=="global"?unique_best(results_,config_.registration):results_.empty()?std::nullopt:std::optional(results_[0]);
      if(best){if(mode_=="global")seed_=best->first;else{bool accepted=policy_.observe(best->first,best->second,latest_.stamp,now().seconds(),mode_=="verify");if(!accepted&&mode_=="local")last_query_=steady()-config_.registration.period+.2;if(mode_=="verify"&&!policy_.confirmations)seed_.reset();if(accepted&&policy_.initialized)seed_.reset();}}
      else{seed_.reset();policy_.reject(mode_=="global"?"全局候选不唯一或配准质量不足":"配准失败",now().seconds());if(mode_=="local")last_query_=steady()-config_.registration.period+.2;}
      batching_=false;return;
    }
    auto guess=guesses_.front();guesses_.pop_front();auto request=std::make_shared<Registration::Request>();request->source=encode(batch_,"navcar_odom",latest_.time);request->initial_guess=to_ros(guess);uint64_t generation=generation_;pending_=true;query_started_=steady();
    auto sent=client_->async_send_request(request,[this,generation](rclcpp::Client<Registration>::SharedFuture future){
      if(generation!=generation_)return;pending_=false;
      try{auto result=future.get();Metrics metrics{result->converged,result->normalized_error,result->overlap,result->min_eigenvalue,result->condition_number};results_.emplace_back(ros_pose(result->transform),metrics);}catch(const std::exception& e){policy_.reject(std::string("配准服务失败：")+e.what(),now().seconds());}submit_next();
    });request_id_=sent.request_id;
  }
  void tick(){
    double clock=now().seconds(),wall=steady();if(pending_&&wall-query_started_>2){client_->remove_pending_request(request_id_);++generation_;pending_=batching_=false;guesses_.clear();policy_.reject("配准计算超时",clock);}
    bool initialized=policy_.initialized;auto state=policy_.status(clock,config_.radius,.02,config_.speed);
    if(initialized&&!policy_.initialized){search_started_=wall;retry_at_=0;seed_.reset();}
    if(!policy_.initialized&&wall-search_started_>config_.registration.search_timeout){retry_at_=wall+config_.registration.retry;search_started_=retry_at_;policy_.reason="全局定位尚不唯一，保持停车后自动重试";}
    if(!pending_&&!batching_&&wall>=retry_at_&&wall-last_query_>=config_.registration.period){last_query_=wall;start_batch();}
    state["stamp_ns"]=Json::Int64(now().nanoseconds());state["backend"]="small_gicp";state["searching"]=!policy_.initialized;
    if(policy_.correction){geometry_msgs::msg::TransformStamped tf;tf.header.frame_id="map";tf.child_frame_id="navcar_odom";tf.header.stamp=now()+rclcpp::Duration::from_seconds(.2);auto pose=to_ros(*policy_.correction);tf.transform.translation.x=pose.position.x;tf.transform.translation.y=pose.position.y;tf.transform.translation.z=pose.position.z;tf.transform.rotation=pose.orientation;broadcaster_->sendTransform(tf);}
    std_msgs::msg::String message;message.data=json_text(state);status_->publish(message);shaoshao_hw_interfaces::msg::LocalizationStatus typed;typed.header.frame_id="map";typed.header.stamp=now();auto phase=state["state"].asString();typed.state=phase=="tracking"?1:phase=="degraded"?2:phase=="lost"?3:0;typed.ready=state["ready"].asBool();typed.speed_limit_mps=state["speed_limit_mps"].asDouble();typed_->publish(typed);
  }
};
}
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<hw_runtime::Localizer>());}catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("hw_localizer"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
