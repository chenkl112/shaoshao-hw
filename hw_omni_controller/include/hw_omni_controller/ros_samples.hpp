#pragma once
#include "feedback.hpp"
#include "model.hpp"
#include <shaoshao_hw_interfaces/msg/chassis_feedback.hpp>
#include <shaoshao_hw_interfaces/msg/predicted_obstacles.hpp>

namespace hw_omni {
inline double seconds(const builtin_interfaces::msg::Time& stamp){return stamp.sec+stamp.nanosec*1e-9;}
inline ChassisSample chassis_sample(const shaoshao_hw_interfaces::msg::ChassisFeedback& msg){
  ChassisSample value;value.stamp=seconds(msg.header.stamp);value.vx=msg.vx_mps;value.vy=msg.vy_mps;
  value.omega=msg.chassis_omega_radps;value.gimbal_yaw=msg.gimbal_relative_yaw_rad;
  value.sequence=msg.seq;value.authority_epoch=msg.authority_epoch;value.control_allowed=msg.control_allowed;value.frame=msg.header.frame_id;
  return value;
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
