#pragma once
#include "model.hpp"
#include <nav_executor/path_executor/monitoring/progress_monitor.hpp>
#include <nav_executor/path_executor/monitoring/safety_monitor.hpp>
#include <nav_executor/path_executor/monitoring/recovery_helpers.hpp>
#include <nav_executor/common/tracking/route_tracker.hpp>
#include <memory>

namespace hw_omni {
inline nav_executor::FsmParams recovery_defaults(double speed=.5){
  return {{.5,.8},
    {30,.06,5,.01,20,.5,1.5,5,72,.05,1,1,5,.3,.2,1,4},
    {std::min(.4,speed*.8),2,.2,std::min(.2,speed*.5),.35,3}};
}
struct RecoveryDecision {
  nav_executor::MotionState state=nav_executor::MotionState::IDLE;
  bool replan=false;
  std::optional<U> target;
};
// HW's state machine, progress monitor, stuck monitor and polar safe-point
// search, with translation directions decoupled from radar/chassis orientation.
class Recovery {
public:
  explicit Recovery(rclcpp::Logger logger,double speed=.5):params_(recovery_defaults(speed)),logger_(logger){reset();}
  void reset(){
    fsm_=std::make_unique<nav_executor::StateMachine>(params_,logger_);
    safety_=std::make_unique<nav_executor::SafetyMonitor>(params_,logger_);
    progress_=std::make_unique<nav_executor::ProgressMonitor>(logger_);
    reverse_direction_=U(-1,0);last_state_=nav_executor::MotionState::IDLE;lethal_count_=0;
  }
  bool active() const {return last_state_==nav_executor::MotionState::STUCK_REVERSE||last_state_==nav_executor::MotionState::HAZARD_RECOVERY;}
  nav_executor::MotionState state() const{return last_state_;}
  bool reject_rollout(){return ++lethal_count_>=3;}
  void accept_rollout(){lethal_count_=0;}
  double reverse_speed() const{return params_.stuck.reverse_speed;}
  RecoveryDecision update(const U& position,const U& measured,const U& previous,const nav_executor::RouteEstimate* route,
      const nav_executor::CostMap& cost,const nav_executor::DirectionMap& direction,std::chrono::steady_clock::time_point stamp){
    using nav_executor::MotionState;
    auto state=fsm_->state();bool was_recovery=active();
    Eigen::Vector3d pose(position.x(),position.y(),0);
    bool no_progress=route&&progress_->update_and_check_no_progress(route->arc_length,{.1,6},state,last_state_,stamp);
    bool stuck=safety_->check_stuck(position,previous.norm(),stamp);
    bool hazard=safety_->compute_is_hazard(cost,direction,position);
    if(state==MotionState::HAZARD_RECOVERY)safety_->update_recovery_goal_if_needed(pose,cost,direction,nullptr,stamp);
    nav_executor::FsmInput input;
    input.has_active_path=route;input.route_tracked=route&&route->status==nav_executor::RouteTrackingStatus::TRACKED;
    input.requested_owner=nav_executor::ControlOwner::NAVIGATION;input.no_progress_detected=no_progress;input.is_stuck=stuck;
    input.is_hazard_now=hazard;input.is_recovery_safe=safety_->check_recovery_safe(cost,direction,position,stamp);
    input.chassis_pos_map=position;input.command_velocity=previous.norm();input.measured_velocity=measured.norm();input.stamp=stamp;
    auto result=fsm_->update(input);
    if(result.state==MotionState::STUCK_REVERSE&&state!=MotionState::STUCK_REVERSE){
      U travel=previous.norm()>.01?previous:(measured.norm()>.01?measured:(route?U(route->reference_position-position):U(1,0)));
      if(travel.norm()<.01&&route)travel=U(std::cos(route->path->trajectory.eval_arc_length(route->arc_length).theta),std::sin(route->path->trajectory.eval_arc_length(route->arc_length).theta));
      reverse_direction_=travel.norm()>1e-8?U(-travel.normalized()):U(-1,0);reverse_origin_=position;
    }
    if(result.state==MotionState::HAZARD_RECOVERY&&state!=MotionState::HAZARD_RECOVERY){safety_->reset_recovery();safety_->update_recovery_goal_if_needed(pose,cost,direction,nullptr,stamp);}
    if(was_recovery&&result.state!=MotionState::STUCK_REVERSE&&result.state!=MotionState::HAZARD_RECOVERY){safety_->reset_stuck();safety_->reset_recovery();progress_->reset();}
    last_state_=result.state;
    RecoveryDecision out;out.state=result.state;out.replan=result.executor_replan_event||(was_recovery&&!active());
    if(result.state==MotionState::STUCK_REVERSE)out.target=reverse_origin_+reverse_direction_*params_.stuck.reverse_displacement;
    if(result.state==MotionState::HAZARD_RECOVERY)out.target=safety_->recovery_goal();
    return out;
  }
private:
  nav_executor::FsmParams params_;rclcpp::Logger logger_;
  std::unique_ptr<nav_executor::StateMachine> fsm_;
  std::unique_ptr<nav_executor::SafetyMonitor> safety_;
  std::unique_ptr<nav_executor::ProgressMonitor> progress_;
  U reverse_direction_=U(-1,0),reverse_origin_=U::Zero();
  nav_executor::MotionState last_state_=nav_executor::MotionState::IDLE;
  unsigned lethal_count_=0;
};
inline const char* motion_name(nav_executor::MotionState state){
  using nav_executor::MotionState;
  switch(state){case MotionState::STUCK_REVERSE:return "stuck_reverse";case MotionState::HAZARD_RECOVERY:return "hazard_recovery";case MotionState::FOLLOW:return "follow";default:return "idle";}
}
}
