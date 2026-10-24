#pragma once
#include "feedback.hpp"
#include "motion.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <shaoshao_hw_interfaces/msg/control_authority.hpp>
#include "model.hpp"
#include <shaoshao_hw_interfaces/msg/predicted_obstacles.hpp>

namespace hw_omni {
inline double seconds(const builtin_interfaces::msg::Time& stamp){return stamp.sec+stamp.nanosec*1e-9;}
inline AuthoritySample authority_sample(const shaoshao_hw_interfaces::msg::ControlAuthority& msg){
  return {seconds(msg.header.stamp),msg.seq,msg.authority_epoch,msg.control_allowed,msg.header.frame_id};
}
inline MotionSample motion_sample(const nav_msgs::msg::Odometry& msg){
  MotionSample out;out.stamp=seconds(msg.header.stamp);out.frame=msg.header.frame_id;out.child=msg.child_frame_id;
  const auto& p=msg.pose.pose.position;const auto& q=msg.pose.pose.orientation;const auto& v=msg.twist.twist.linear;const auto& w=msg.twist.twist.angular;
  out.position={p.x,p.y,p.z};out.orientation=Eigen::Quaterniond(q.w,q.x,q.y,q.z);out.velocity={v.x,v.y,v.z};out.omega={w.x,w.y,w.z};
  out.covariance=Eigen::Map<const Eigen::Matrix<double,6,6,Eigen::RowMajor>>(msg.twist.covariance.data());
  for(double value:msg.pose.covariance)if(!std::isfinite(value)){out.velocity.x()=std::numeric_limits<double>::quiet_NaN();break;}return out;
}
// FAST-LIO reports the IMU origin and body axes. navcar_center has parallel
// axes; compensate its fixed lever arm without differentiating positions.
inline nav_msgs::msg::Odometry center_odometry(nav_msgs::msg::Odometry msg,const Eigen::Vector3d& offset){
  if(msg.header.frame_id!="navcar_odom"||msg.child_frame_id!="navcar_imu"||!offset.allFinite())throw std::invalid_argument("Unexpected FAST-LIO frame");
  auto state=motion_sample(msg);
  if(!state.position.allFinite()||!state.orientation.coeffs().allFinite()||std::abs(state.orientation.norm()-1)>.001||!state.velocity.allFinite()||!state.omega.allFinite())throw std::invalid_argument("Invalid FAST-LIO state");
  Eigen::Vector3d shift=state.orientation*offset,position=state.position+shift,velocity=state.velocity+state.omega.cross(offset);
  msg.pose.pose.position.x=position.x();msg.pose.pose.position.y=position.y();msg.pose.pose.position.z=position.z();msg.child_frame_id="navcar_center";
  msg.twist.twist.linear.x=velocity.x();msg.twist.twist.linear.y=velocity.y();msg.twist.twist.linear.z=velocity.z();
  auto skew=[](const Eigen::Vector3d& p){Eigen::Matrix3d m;m<<0,-p.z(),p.y(),p.z(),0,-p.x(),-p.y(),p.x(),0;return m;};
  Eigen::Matrix<double,6,6> j=Eigen::Matrix<double,6,6>::Identity();j.block<3,3>(0,3)=-skew(offset);
  Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> tc(msg.twist.covariance.data());tc=(j*tc*j.transpose()).eval();
  j.setIdentity();j.block<3,3>(0,3)=-skew(shift);
  Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> pc(msg.pose.covariance.data());pc=(j*pc*j.transpose()).eval();return msg;
}
inline std::vector<Obstacle> read_predictions(const shaoshao_hw_interfaces::msg::PredictedObstacles& msg,double now){
  double age=now-seconds(msg.header.stamp);
  if(!msg.valid||msg.header.frame_id!="map"||age<-.05||age>.3||!std::isfinite(msg.prediction_dt)||
     msg.prediction_dt<.02||msg.prediction_dt>.2||msg.obstacles.size()>128)
    throw std::invalid_argument("Invalid obstacle prediction header");
  std::vector<Obstacle> result;
  for(const auto& item:msg.obstacles){
    if(item.centers.size()<2||item.centers.size()>256||item.footprint.points.size()<3||item.footprint.points.size()>512||
       (item.centers.size()-1)*msg.prediction_dt<std::max(0.0,age)+N*DT)
      throw std::invalid_argument("Invalid obstacle shape or horizon");
    Obstacle obstacle;obstacle.dt=msg.prediction_dt;obstacle.age=std::max(0.0,age);
    for(const auto& p:item.centers){if(!std::isfinite(p.x)||!std::isfinite(p.y))throw std::invalid_argument("Nonfinite obstacle center");obstacle.centers.emplace_back(p.x,p.y);}
    for(const auto& p:item.footprint.points){if(!std::isfinite(p.x)||!std::isfinite(p.y))throw std::invalid_argument("Nonfinite obstacle shape");obstacle.footprint.emplace_back(p.x,p.y);}
    result.push_back(std::move(obstacle));
  }
  return result;
}
}
