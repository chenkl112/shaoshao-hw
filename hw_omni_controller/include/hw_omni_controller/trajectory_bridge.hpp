#pragma once
#include <shaoshao_hw_interfaces/msg/planned_trajectory.hpp>
#include <nav_executor/common/trajectory/trajectory_validation.hpp>
#include <nav_executor/common/trajectory/annotated_path.hpp>
#include "model.hpp"

namespace hw_omni {
using TrajectoryMessage=shaoshao_hw_interfaces::msg::PlannedTrajectory;
inline bool same_plan(const nav_msgs::msg::Path& a,const nav_msgs::msg::Path& b){
  if(a.header!=b.header||a.poses.size()!=b.poses.size()||a.poses.empty())return false;
  for(size_t i=0;i<a.poses.size();++i)if(a.poses[i].pose!=b.poses[i].pose)return false;
  return true;
}
inline TrajectoryMessage encode_trajectory(const nav_executor::AnnotatedPath& plan,const nav_msgs::msg::Path& path){
  TrajectoryMessage out;out.path=path;out.durations=plan.trajectory.durations();
  for(const auto& block:plan.trajectory.coefficients())for(int k=0;k<6;++k)for(int j=0;j<2;++j)out.coefficients.push_back(block(k,j));
  for(const auto& state:plan.speed_profile.states()){out.profile_s.push_back(state.arc_length);out.profile_time.push_back(state.time);out.profile_speed.push_back(state.velocity);}
  return out;
}
inline nav_executor::AnnotatedPath::ConstPtr decode_trajectory(const TrajectoryMessage& msg){
  auto n=msg.durations.size(),m=msg.profile_s.size();
  if(!n||n>2000||msg.coefficients.size()!=n*12||m<2||m>100000||msg.profile_time.size()!=m||msg.profile_speed.size()!=m||msg.path.header.frame_id!="map"||msg.path.poses.empty()||msg.path.poses.size()>100000)
    throw std::invalid_argument("Invalid HW trajectory dimensions");
  std::vector<nav_executor::MincoTrajectory::CoefBlock> blocks(n);
  for(size_t i=0;i<n;++i){if(!std::isfinite(msg.durations[i])||msg.durations[i]<=0||msg.durations[i]>1e6)throw std::invalid_argument("Invalid MINCO interval");
    for(int k=0;k<6;++k)for(int j=0;j<2;++j){double v=msg.coefficients[i*12+k*2+j];if(!std::isfinite(v))throw std::invalid_argument("Nonfinite MINCO coefficient");blocks[i](k,j)=v;}}
  auto out=std::make_shared<nav_executor::AnnotatedPath>(nav_executor::MincoTrajectory(msg.durations,std::move(blocks),{}));
  if(auto error=nav_executor::validate_trajectory_numerics(out->trajectory))throw std::invalid_argument(*error);
  std::vector<nav_executor::SpeedProfileState> states;
  for(size_t i=0;i<m;++i){double s=msg.profile_s[i],t=msg.profile_time[i],v=msg.profile_speed[i];
    if(!std::isfinite(s)||!std::isfinite(t)||!std::isfinite(v)||s<0||t<0||v<0||v>.500001||(i&&(s<=msg.profile_s[i-1]||t<=msg.profile_time[i-1])))throw std::invalid_argument("Invalid HW speed profile");
    states.push_back({s,t,v});}
  if(std::abs(states.front().arc_length)>1e-6||std::abs(states.front().time)>1e-6||std::abs(states.back().arc_length-out->trajectory.total_arc_length())>1e-4)throw std::invalid_argument("HW geometry/profile mismatch");
  out->speed_profile=nav_executor::PathSpeedProfile(std::move(states));
  const auto& end=msg.path.poses.back().pose.position;out->goal_pos=U(end.x,end.y);
  if((out->trajectory.eval_arc_length(out->trajectory.total_arc_length()).p-out->goal_pos).norm()>.001)throw std::invalid_argument("HW endpoint mismatch");
  return out;
}
}
