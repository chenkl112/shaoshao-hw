#include "hw_omni_controller/obstacles.hpp"
#include "hw_omni_controller/cloud.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shaoshao_hw_interfaces/msg/localization_status.hpp>
#include <shaoshao_hw_interfaces/msg/predicted_obstacles.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sstream>

namespace {
double seconds(const builtin_interfaces::msg::Time& stamp){return stamp.sec+stamp.nanosec*1e-9;}
bool fresh(double now,double stamp,double timeout){return now-stamp>=-.05&&now-stamp<=timeout;}
Eigen::Isometry3d transform(const geometry_msgs::msg::Transform& msg){
  Eigen::Quaterniond q(msg.rotation.w,msg.rotation.x,msg.rotation.y,msg.rotation.z);
  if(!q.coeffs().allFinite()||std::abs(q.norm()-1)>.01)throw std::invalid_argument("Invalid transform rotation");
  Eigen::Isometry3d result=Eigen::Isometry3d::Identity();result.linear()=q.normalized().toRotationMatrix();
  result.translation()<<msg.translation.x,msg.translation.y,msg.translation.z;
  if(!result.matrix().allFinite())throw std::invalid_argument("Invalid transform translation");
  return result;
}
}

class ObstaclePredictor : public rclcpp::Node {
public:
  ObstaclePredictor():Node("hw_obstacle_predictor"){
    hw_obstacles::Settings settings;
    settings.resolution=declare_parameter("resolution_m",.05);
    settings.range=declare_parameter("range_m",8.);
    settings.static_distance=declare_parameter("static_distance_m",.15);
    settings.association_gate=declare_parameter("association_gate_m",.8);
    settings.confirmations=declare_parameter("confirmations",3);
    settings.lost_timeout=declare_parameter("lost_timeout_s",.5);
    settings.velocity_decay=declare_parameter("velocity_decay_s",2.);
    settings.validate();detector_=std::make_unique<hw_obstacles::Detector>(settings);
    tracker_=std::make_unique<hw_obstacles::Tracker>(settings);
    lidar_height_=declare_parameter("lidar_height_m",.327);
    if(!std::isfinite(lidar_height_)||lidar_height_<=0)throw std::invalid_argument("Invalid lidar height");
    auto map_path=declare_parameter<std::string>("map_path","");
    auto loaded=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if(pcl::io::loadPCDFile(map_path,*loaded)<0)throw std::runtime_error("Cannot load static obstacle map");
    map_=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();map_->reserve(loaded->size());
    for(const auto& p:*loaded)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))map_->push_back(p);
    if(map_->empty())throw std::runtime_error("Empty static obstacle map");
    tree_.setInputCloud(map_);
    buffer_=std::make_shared<tf2_ros::Buffer>(get_clock());
    // Dedicated listener thread services TF while a cloud waits at its sample time.
    listener_=std::make_shared<tf2_ros::TransformListener>(*buffer_);
    prediction_pub_=create_publisher<shaoshao_hw_interfaces::msg::PredictedObstacles>("/hw/predicted_obstacles",1);
    status_pub_=create_publisher<std_msgs::msg::String>("/hw/obstacles/status",1);
    quality_sub_=create_subscription<shaoshao_hw_interfaces::msg::LocalizationStatus>("/hw/localization/state",10,
      [this](shaoshao_hw_interfaces::msg::LocalizationStatus::SharedPtr msg){
        double stamp=seconds(msg->header.stamp);
        if(stamp<=quality_stamp_)return;
        bool valid=msg->header.frame_id=="map"&&fresh(now().seconds(),stamp,.5);
        localized_=valid&&msg->ready&&(msg->state==1||msg->state==2);
        if(valid)quality_stamp_=stamp;
        if(!localized_)tracker_->reset();
      });
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered",rclcpp::SensorDataQoS(),
      std::bind(&ObstaclePredictor::cloud,this,std::placeholders::_1));
  }
private:
  std::unique_ptr<hw_obstacles::Detector> detector_;std::unique_ptr<hw_obstacles::Tracker> tracker_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_;pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<shaoshao_hw_interfaces::msg::PredictedObstacles>::SharedPtr prediction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<shaoshao_hw_interfaces::msg::LocalizationStatus>::SharedPtr quality_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  bool localized_=false;double quality_stamp_=0,last_stamp_=0,lidar_height_=.327;
  std::optional<Eigen::Isometry3d> correction_;

  void cloud(sensor_msgs::msg::PointCloud2::SharedPtr msg){
    shaoshao_hw_interfaces::msg::PredictedObstacles output;output.header.frame_id="map";output.header.stamp=msg->header.stamp;output.prediction_dt=.1;
    std::string reason;
    try{
      double stamp=seconds(msg->header.stamp),current=now().seconds();
      if(!fresh(current,stamp,.3)||msg->header.frame_id!="navcar_odom")throw std::runtime_error("Stale cloud or unexpected frame");
      if(stamp<=last_stamp_)return;
      last_stamp_=stamp;
      if(!localized_||!fresh(current,quality_stamp_,.5))throw std::runtime_error("Waiting for current map localization");
      std::array<hw_obstacles::FloatField,3> fields;std::array<bool,3> found{};
      for(const auto& f:msg->fields)for(int i=0;i<3;++i)if(f.name==std::array<std::string,3>{"x","y","z"}[i]){
        if(f.count!=1)throw std::runtime_error("XYZ fields must be scalar");
        fields[i]={f.offset,f.datatype};found[i]=true;
      }
      if(!std::all_of(found.begin(),found.end(),[](bool value){return value;}))throw std::runtime_error("Missing XYZ fields");
      auto points=hw_obstacles::decode_cloud(msg->data,msg->width,msg->height,msg->point_step,msg->row_step,msg->is_bigendian,fields);
      if(points.size()<50)throw std::runtime_error("Insufficient obstacle cloud returns");
      auto tf=buffer_->lookupTransform("map","navcar_odom",rclcpp::Time(msg->header.stamp),rclcpp::Duration::from_seconds(.02));
      auto correction=transform(tf.transform);
      if(correction_){
        double translation=(correction.translation()-correction_->translation()).norm();
        double angle=Eigen::AngleAxisd(correction_->linear().transpose()*correction.linear()).angle();
        if(translation>.10||angle>.05)tracker_->reset();
      }
      correction_=correction;
      auto lidar=buffer_->lookupTransform("map","navcar_lidar",rclcpp::Time(msg->header.stamp),rclcpp::Duration::from_seconds(.02));
      auto sensor=transform(lidar.transform);
      for(auto& p:points)p=correction*p;
      auto nearest=[this](const hw_obstacles::CloudPoint& p){
        pcl::PointXYZ point;point.x=static_cast<float>(p.x());point.y=static_cast<float>(p.y());point.z=static_cast<float>(p.z());
        std::vector<int> index(1);std::vector<float> distance(1);
        if(tree_.nearestKSearch(point,1,index,distance)!=1)throw std::runtime_error("Static map query failed");
        return std::sqrt(static_cast<double>(distance[0]));
      };
      auto detections=detector_->detect(points,sensor.translation().head<2>(),sensor.translation().z()-lidar_height_,nearest);
      tracker_->update(detections,stamp);
      for(const auto& prediction:tracker_->predict()){
        shaoshao_hw_interfaces::msg::PredictedObstacle item;item.id=prediction.id;item.confirmed=prediction.confirmed;
        for(const auto& p:prediction.centers){geometry_msgs::msg::Point center;center.x=p.x();center.y=p.y();item.centers.push_back(center);}
        for(const auto& p:prediction.footprint){geometry_msgs::msg::Point32 vertex;vertex.x=static_cast<float>(p.x());vertex.y=static_cast<float>(p.y());item.footprint.points.push_back(vertex);}
        output.obstacles.push_back(std::move(item));
      }
      output.valid=true;
    }catch(const std::exception& e){tracker_->reset();reason=e.what();}
    prediction_pub_->publish(output);
    std_msgs::msg::String status;std::ostringstream text;
    // Reasons are emitted as ROS logs; JSON carries only bounded native values.
    text<<"{\"valid\":"<<(output.valid?"true":"false")<<",\"tracks\":"<<output.obstacles.size()<<"}";
    status.data=text.str();status_pub_->publish(status);
    if(!reason.empty())RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"%s",reason.c_str());
  }
};
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<ObstaclePredictor>());}catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("hw_obstacle_predictor"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
