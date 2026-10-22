#pragma once
#include "common.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <hw_omni_controller/cloud.hpp>
#include <thread>

namespace hw_runtime {
class SpinThread {
public:
  explicit SpinThread(const rclcpp::Node::SharedPtr& node):node_(node),executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(),2)){
    executor_->add_node(node_);thread_=std::thread([this]{executor_->spin();});
  }
  ~SpinThread(){executor_->cancel();if(thread_.joinable())thread_.join();executor_->remove_node(node_);}
private:rclcpp::Node::SharedPtr node_;std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;std::thread thread_;
};
inline double seconds(const builtin_interfaces::msg::Time& stamp){return stamp.sec+stamp.nanosec*1e-9;}
inline bool fresh(double now,const builtin_interfaces::msg::Time& stamp,double timeout){double age=now-seconds(stamp);return age>=-.05&&age<=timeout;}
inline double sample_steady(const builtin_interfaces::msg::Time& stamp,double ros_now,double mono_now,double timeout){double age=ros_now-seconds(stamp);return age>=-.05&&age<=timeout?mono_now-std::max(0.,age):0.;}
inline Pose ros_pose(const geometry_msgs::msg::Pose& msg){
  Eigen::Quaterniond q(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
  if(!q.coeffs().allFinite()||std::abs(q.norm()-1)>.01)throw std::invalid_argument("Invalid ROS quaternion");Pose p=Pose::Identity();p.linear()=q.normalized().toRotationMatrix();p.translation()<<msg.position.x,msg.position.y,msg.position.z;if(!p.translation().allFinite())throw std::invalid_argument("Invalid ROS position");return p;
}
inline Pose ros_transform(const geometry_msgs::msg::Transform& msg){geometry_msgs::msg::Pose pose;pose.position.x=msg.translation.x;pose.position.y=msg.translation.y;pose.position.z=msg.translation.z;pose.orientation=msg.rotation;return ros_pose(pose);}
inline geometry_msgs::msg::Pose to_ros(const Pose& p){geometry_msgs::msg::Pose msg;msg.position.x=p.translation().x();msg.position.y=p.translation().y();msg.position.z=p.translation().z();Eigen::Quaterniond q(p.linear());msg.orientation.x=q.x();msg.orientation.y=q.y();msg.orientation.z=q.z();msg.orientation.w=q.w();return msg;}
inline Cloud decode(const sensor_msgs::msg::PointCloud2& msg){
  std::array<hw_obstacles::FloatField,3> fields;std::array<bool,3> found{};const std::array<std::string,3> names{"x","y","z"};
  for(const auto& f:msg.fields)for(int i=0;i<3;++i)if(f.name==names[i]&&f.count==1){fields[i]={f.offset,f.datatype};found[i]=true;}
  if(!std::all_of(found.begin(),found.end(),[](bool value){return value;}))throw std::invalid_argument("Missing XYZ cloud fields");
  return hw_obstacles::decode_cloud(msg.data,msg.width,msg.height,msg.point_step,msg.row_step,msg.is_bigendian,fields);
}
inline sensor_msgs::msg::PointCloud2 encode(const Cloud& points,const std::string& frame,const builtin_interfaces::msg::Time& stamp){
  sensor_msgs::msg::PointCloud2 msg;msg.header.frame_id=frame;msg.header.stamp=stamp;msg.height=1;msg.width=static_cast<uint32_t>(points.size());msg.point_step=12;msg.row_step=12*msg.width;msg.is_bigendian=false;msg.is_dense=true;msg.data.reserve(msg.row_step);
  const std::array<std::string,3> names{"x","y","z"};for(int i=0;i<3;++i){sensor_msgs::msg::PointField field;field.name=names[i];field.offset=4*i;field.datatype=7;field.count=1;msg.fields.push_back(field);}
  for(const auto& p:points)for(int i=0;i<3;++i){uint32_t bits=std::bit_cast<uint32_t>(static_cast<float>(p[i]));for(int b=0;b<4;++b)msg.data.push_back(static_cast<uint8_t>((bits>>(8*b))&255));}return msg;
}
}
